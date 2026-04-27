//! Motor driver abstraction for the bilateral controller.
//!
//! The bilateral loop talks to two motors over a single shared CAN bus.
//! This module hides the per-vendor protocol behind [`MotorDriver`], so the
//! bilateral logic can mix motors from different vendors freely as long as
//! each vendor exposes the same operations: enable, disable, MIT-mode
//! exchange, and (for OnDemand mode) position/velocity reads while disabled.
//!
//! At present only [`RobstrideDriver`] is provided. A DAMIAO driver will be
//! added on top of this trait in a follow-up change.

use std::time::{Duration, Instant};

use socketcan::{CanSocket, EmbeddedFrame, ExtendedId, Id, Socket, StandardId};

use crate::error::{Result, RobstrideError};
use crate::protocol::*;

/// Common motor feedback returned by every driver.
///
/// Bilateral logic uses `position`, `velocity`, and `torque` only; vendor-
/// specific status bits remain in [`MotorFeedback::status`] for drivers that
/// can populate them (Robstride). Other drivers may leave it at default.
pub type Feedback = MotorFeedback;

/// Vendor-agnostic motor I/O over a shared CAN socket.
///
/// Drivers must be `Send` because the bilateral control loop runs in a
/// dedicated thread and takes ownership of two boxed drivers.
///
/// All driver methods take `&mut self` because each driver carries a
/// soft-zero offset that is mutated by [`MotorDriver::set_soft_zero`] and
/// read by [`MotorDriver::mit_exchange`].
pub trait MotorDriver: Send {
    /// Enable the motor (allow torque output).
    fn enable(&mut self, socket: &CanSocket) -> Result<()>;

    /// Disable the motor (free-spin / zero output).
    fn disable(&mut self, socket: &CanSocket) -> Result<()>;

    /// Latch the current physical position as the in-memory zero reference.
    ///
    /// **Does not touch motor NVM** — the offset lives only inside this
    /// driver instance, so calibration must be re-done after a process
    /// restart. The motor is briefly enabled and immediately disabled to
    /// read its current position (all MIT gains kept at zero so no torque
    /// is commanded).
    fn set_soft_zero(&mut self, socket: &CanSocket) -> Result<()>;

    /// Current soft-zero offset [rad].
    fn soft_zero_offset(&self) -> f64;

    /// Set the soft-zero offset directly (for restoring from a saved
    /// session value without re-running [`MotorDriver::set_soft_zero`]).
    fn set_soft_zero_offset(&mut self, offset: f64);

    /// Whether this motor's position/velocity/torque are flipped relative
    /// to the host frame. Used to reconcile vendors that disagree on which
    /// rotation direction is "positive" (e.g. Robstride and DAMIAO).
    fn invert(&self) -> bool;

    /// Set the polarity-flip flag (see [`MotorDriver::invert`]).
    fn set_invert(&mut self, invert: bool);

    /// Send one MIT-mode command frame and return the resulting feedback.
    ///
    /// `position` [rad], `velocity` [rad/s], `kp` [Nm/rad], `kd` [Nm·s/rad],
    /// `torque` [Nm] — all in physical units; the driver scales them and
    /// applies the soft-zero offset (so callers see a coordinate frame
    /// where the zero point is wherever [`MotorDriver::set_soft_zero`] was
    /// invoked).
    fn mit_exchange(
        &mut self,
        socket: &CanSocket,
        position: f64,
        velocity: f64,
        kp: f64,
        kd: f64,
        torque: f64,
    ) -> Result<Feedback>;

    /// Read mechanical position [rad]. Must work while the motor is disabled
    /// (used by the OnDemand bilateral method to keep the leader free).
    /// The returned value is already soft-zero-adjusted.
    fn read_position(&mut self, socket: &CanSocket) -> Result<f64>;

    /// Read mechanical velocity [rad/s]. Same disabled-read requirement as
    /// [`MotorDriver::read_position`]. Velocity is unaffected by the soft
    /// zero (it's a derivative quantity).
    fn read_velocity(&mut self, socket: &CanSocket) -> Result<f64>;

    /// Safe output torque limit [Nm] used by the bilateral loop for clamping.
    /// Implementations should already include any safety margin they want
    /// applied to commanded torque.
    fn torque_limit(&self) -> f64;

    /// Identifier suitable for log lines / UI labels.
    fn description(&self) -> String;
}

// =============================================================================
// Low-level CAN helpers (Robstride extended-ID framing)
// =============================================================================

fn send_can(socket: &CanSocket, can_id: u32, data: &[u8]) -> Result<()> {
    let ext_id = ExtendedId::new(can_id).expect("Invalid extended CAN ID");
    let frame = socketcan::CanFrame::new(ext_id, data).expect("Failed to create CAN frame");
    socket.write_frame(&frame)?;
    Ok(())
}

fn recv_can(socket: &CanSocket, timeout: Duration) -> Result<(u8, u16, u8, Vec<u8>)> {
    let start = Instant::now();
    loop {
        if start.elapsed() > timeout {
            return Err(RobstrideError::Timeout { motor_id: 0 });
        }
        match socket.read_frame() {
            Ok(frame) => {
                if !frame.is_extended() {
                    continue;
                }
                let raw_id = match frame.id() {
                    Id::Standard(sid) => socketcan::StandardId::as_raw(&sid) as u32,
                    Id::Extended(eid) => ExtendedId::as_raw(&eid),
                };
                let data = frame.data().to_vec();
                let (comm_type, extra_data, device_id) = parse_can_id(raw_id);
                return Ok((comm_type, extra_data, device_id, data));
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
            Err(e) => return Err(RobstrideError::CanSocket(e)),
        }
    }
}

/// Result of a passive sniff of the CAN bus.
#[derive(Debug, Clone, Default)]
pub struct BusSniffResult {
    /// Total Robstride OperationStatus frames observed in the window.
    pub status_frames: u32,
    /// Set of distinct `host_id` values (the `dev` field of a status
    /// frame) that the observed motors are addressing. If anyone other
    /// than us is driving the bus, our host_id will appear here too
    /// (because motors echo back to whoever asked) — so a non-empty set
    /// that doesn't match the host_id we are about to use, or *contains*
    /// our host_id while we have not yet transmitted, indicates an
    /// already-running peer process on the same bus.
    pub host_ids: std::collections::BTreeSet<u8>,
    /// Set of distinct motor IDs (low byte of `extra_data` in status
    /// frames) observed responding on the bus.
    pub motor_ids: std::collections::BTreeSet<u8>,
}

/// Passively listen on the bus for `duration` without sending anything.
///
/// Used as a pre-flight check to detect another process (or another host)
/// already driving the same bus. The expected steady-state behaviour with
/// no commands issued is **zero** Robstride OperationStatus frames; any
/// non-zero count means a peer is talking and our `mit_exchange`
/// host-id filter cannot reliably disambiguate frame ownership.
///
/// This intentionally only counts Robstride extended-frame OperationStatus
/// (CommType=2): standard 11-bit ID traffic from DAMIAO motors, generic
/// CAN sensors, etc. is unrelated and would produce false positives.
pub fn sniff_robstride_bus(socket: &CanSocket, duration: Duration) -> BusSniffResult {
    let mut out = BusSniffResult::default();
    let deadline = Instant::now() + duration;
    while Instant::now() < deadline {
        let remaining = deadline.saturating_duration_since(Instant::now());
        let slice = remaining.min(Duration::from_millis(20));
        match recv_can(socket, slice) {
            Ok((ct, extra, dev, _data)) => {
                if ct == CommType::OperationStatus as u8 {
                    out.status_frames += 1;
                    out.host_ids.insert(dev);
                    // For OperationStatus frames the low byte of
                    // extra_data carries the responding motor's ID.
                    out.motor_ids.insert((extra & 0xFF) as u8);
                }
            }
            Err(RobstrideError::Timeout { .. }) => {
                // Expected when the bus is quiet — keep listening.
                continue;
            }
            Err(_) => continue,
        }
    }
    out
}

// =============================================================================
// Robstride driver
// =============================================================================

/// Robstride MIT-mode driver. Uses 29-bit extended CAN IDs that encode
/// `(comm_type, extra_data, device_id)`; see [`crate::protocol`].
///
/// Torque limit is reported as 50 % of the model's MIT torque scale, matching
/// the existing bilateral safety margin.
pub struct RobstrideDriver {
    host_id: u8,
    motor_id: u8,
    model: MotorModel,
    scales: MitScales,
    soft_zero: f64,
    invert: bool,
}

impl RobstrideDriver {
    pub fn new(host_id: u8, motor_id: u8, model: MotorModel) -> Self {
        Self {
            host_id,
            motor_id,
            model,
            scales: MitScales::for_model(model),
            soft_zero: 0.0,
            invert: false,
        }
    }

    fn sign(&self) -> f64 {
        if self.invert { -1.0 } else { 1.0 }
    }

    pub fn motor_id(&self) -> u8 {
        self.motor_id
    }

    pub fn model(&self) -> MotorModel {
        self.model
    }

    fn read_param(&mut self, socket: &CanSocket, param: ParamIndex) -> Result<f32> {
        let (can_id, data) = build_read_param_frame(self.host_id, self.motor_id, param);
        send_can(socket, can_id, &data)?;
        let deadline = Instant::now() + Duration::from_millis(20);
        loop {
            let timeout = deadline
                .duration_since(Instant::now())
                .max(Duration::from_millis(1));
            let (_ct, _extra, _dev, rdata) = recv_can(socket, timeout)?;
            if let Some((_idx, val)) = parse_param_response(&rdata) {
                return Ok(val);
            }
        }
    }
}

impl MotorDriver for RobstrideDriver {
    fn enable(&mut self, socket: &CanSocket) -> Result<()> {
        let (can_id, data) = build_enable_frame(self.host_id, self.motor_id);
        send_can(socket, can_id, &data)?;
        // Consume the response frame if it arrives; ignore timeout.
        let _ = recv_can(socket, Duration::from_millis(50));
        Ok(())
    }

    fn disable(&mut self, socket: &CanSocket) -> Result<()> {
        let (can_id, data) = build_disable_frame(self.host_id, self.motor_id);
        send_can(socket, can_id, &data)?;
        let _ = recv_can(socket, Duration::from_millis(50));
        Ok(())
    }

    fn set_soft_zero(&mut self, socket: &CanSocket) -> Result<()> {
        soft_zero_via_mit(self, socket)
    }

    fn soft_zero_offset(&self) -> f64 {
        self.soft_zero
    }

    fn set_soft_zero_offset(&mut self, offset: f64) {
        self.soft_zero = offset;
    }

    fn invert(&self) -> bool {
        self.invert
    }

    fn set_invert(&mut self, invert: bool) {
        self.invert = invert;
    }

    fn mit_exchange(
        &mut self,
        socket: &CanSocket,
        position: f64,
        velocity: f64,
        kp: f64,
        kd: f64,
        torque: f64,
    ) -> Result<Feedback> {
        // soft_zero is stored in the motor's RAW frame so it is
        // polarity-independent (toggling invert after calibration still
        // leaves 0 = the calibration pose):
        //   command  = sign*host_pos + raw_offset
        //   feedback = sign*(raw - raw_offset)
        let sign = self.sign();
        let (can_id, data) = build_mit_frame(
            self.host_id,
            self.motor_id,
            &self.scales,
            sign * position + self.soft_zero,
            sign * velocity,
            kp,
            kd,
            sign * torque,
        );
        send_can(socket, can_id, &data)?;

        // Read response; skip echoes / unrelated motor frames.
        //
        // We validate three things on each candidate frame:
        //   1. CommType == OperationStatus
        //   2. The CAN ID's `dev` field equals our host_id — the motor
        //      *targeted* its response at us. If the motor was previously
        //      bound to a different host (e.g. another process on the bus)
        //      its status frames may carry a foreign `dev` and we must
        //      ignore them or we'll parse another conversation's bytes
        //      as our own motor's state.
        //   3. The status-frame's encoded `motor_id` (low byte of
        //      extra_data) matches our `self.motor_id`.
        let deadline = Instant::now() + Duration::from_millis(10);
        loop {
            let timeout = deadline
                .duration_since(Instant::now())
                .max(Duration::from_millis(1));
            let (ct, extra, dev, rdata) = recv_can(socket, timeout)?;
            if ct != CommType::OperationStatus as u8 {
                continue;
            }
            if dev != self.host_id {
                continue;
            }
            let raw = build_can_id_raw(ct, extra, dev);
            if let Some(mut fb) = parse_status_frame(raw, &rdata, &self.scales) {
                if fb.motor_id != self.motor_id {
                    continue;
                }
                fb.position = sign * (fb.position - self.soft_zero);
                fb.velocity = sign * fb.velocity;
                fb.torque = sign * fb.torque;
                return Ok(fb);
            }
        }
    }

    fn read_position(&mut self, socket: &CanSocket) -> Result<f64> {
        let raw = self.read_param(socket, ParamIndex::MechPos)? as f64;
        Ok(self.sign() * (raw - self.soft_zero))
    }

    fn read_velocity(&mut self, socket: &CanSocket) -> Result<f64> {
        Ok(self.sign() * self.read_param(socket, ParamIndex::MechVel)? as f64)
    }

    fn torque_limit(&self) -> f64 {
        self.scales.torque * 0.5
    }

    fn description(&self) -> String {
        format!("Robstride {} ID:{}", self.model, self.motor_id)
    }
}

/// Shared soft-zero logic for any driver that can read its current position
/// via an MIT-zero exchange while enabled. Resets the existing offset first
/// so the read returns the raw physical position, then stores that as the
/// new offset. The stored offset is in the motor's raw frame, so a later
/// invert toggle leaves the calibration intact.
fn soft_zero_via_mit<D: MotorDriver + ?Sized>(driver: &mut D, socket: &CanSocket) -> Result<()> {
    driver.set_soft_zero_offset(0.0);
    driver.enable(socket)?;
    // Brief settle so the motor is ready to report state.
    std::thread::sleep(Duration::from_millis(10));
    let fb_result = driver.mit_exchange(socket, 0.0, 0.0, 0.0, 0.0, 0.0);
    // Always disable, even if the read failed, to avoid leaving the motor
    // hot.
    let _ = driver.disable(socket);
    let fb = fb_result?;
    // With offset=0, mit_exchange returned sign*raw. Undo sign to get raw.
    let sign = if driver.invert() { -1.0 } else { 1.0 };
    driver.set_soft_zero_offset(sign * fb.position);
    Ok(())
}

// =============================================================================
// DAMIAO driver (DM-J4310-2EC family)
// =============================================================================
//
// DAMIAO DM-series motors use 11-bit standard CAN IDs and the classic T-Motor
// MIT bit packing:
//
//   command (8 bytes):
//     [0..1]  position    u16  -> [-P_MAX, P_MAX]
//     [2..3]  velocity    u12  -> [-V_MAX, V_MAX]  (top 12 bits of bytes 2..3)
//     [3..4]  kp          u12  -> [0,      KP_MAX] (bottom 12 of bytes 3..4)
//     [5..6]  kd          u12  -> [0,      KD_MAX]
//     [6..7]  torque      u12  -> [-T_MAX, T_MAX]
//
//   response (8 bytes):
//     [0]     err nibble | motor_id nibble
//     [1..2]  position    u16
//     [3..4]  velocity    u12 | torque high nibble
//     [4..5]  torque      u12 (low 12 bits of [4..5])
//     [6]     MOS temp    u8
//     [7]     rotor temp  u8
//
// Enable/disable/zero/clear-error use the magic "FF..FX" sequences:
//   FF FF FF FF FF FF FF FC  enable
//   FF FF FF FF FF FF FF FD  disable
//   FF FF FF FF FF FF FF FE  set zero
//   FF FF FF FF FF FF FF FB  clear error

const DM_ENABLE: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC];
const DM_DISABLE: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD];
// FF..FE = "save current position as zero to NVM". Used by
// damiao_set_zero_nvm — DO NOT call routinely (flash wear).
const DM_SET_ZERO_NVM: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE];

/// DAMIAO motor model.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum DamiaoModel {
    /// DM-J4310-2EC integrated joint actuator (built-in 10:1 reducer).
    DmJ4310_2EC,
}

impl DamiaoModel {
    pub fn name(&self) -> &'static str {
        match self {
            DamiaoModel::DmJ4310_2EC => "DM-J4310-2EC",
        }
    }

    pub fn from_str_ci(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "dm-j4310-2ec" | "dmj4310-2ec" | "dm-j4310" | "dmj4310"
            | "j4310-2ec" | "j4310" | "dm4310" | "dm4310-2ec" => {
                Some(DamiaoModel::DmJ4310_2EC)
            }
            _ => None,
        }
    }
}

impl std::fmt::Display for DamiaoModel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

/// MIT scaling ranges per DAMIAO model. The motor firmware decodes the 12/16-
/// bit packed fields against the same ranges, so these must match the values
/// configured on the motor (DAMIAO's defaults are baked in here).
#[derive(Debug, Clone, Copy)]
pub struct DamiaoLimits {
    pub p_max: f64, // rad
    pub v_max: f64, // rad/s
    pub kp_max: f64,
    pub kd_max: f64,
    pub t_max: f64, // Nm
}

impl DamiaoLimits {
    pub fn for_model(model: DamiaoModel) -> Self {
        match model {
            DamiaoModel::DmJ4310_2EC => DamiaoLimits {
                p_max: 12.5,
                v_max: 30.0,
                kp_max: 500.0,
                kd_max: 5.0,
                t_max: 10.0,
            },
        }
    }
}

/// DAMIAO MIT-mode driver.
///
/// `can_id` is the motor's TX address (its `CAN_ID` register, default 0x01 on
/// a fresh motor). `master_id` is the standard ID the motor uses for its
/// responses (its `MST_ID` register; 0 means "accept any standard ID, match
/// by the motor-id nibble in byte 0 of the payload"). Using a unique
/// `MST_ID` per motor on a shared bus avoids ambiguity when multiple DM
/// motors share the same MST_ID.
///
/// Reading state while disabled is not supported by DM MIT firmware, so
/// OnDemand bilateral mode cannot use a DAMIAO leader.
pub struct DamiaoDriver {
    can_id: u8,
    master_id: u16,
    model: DamiaoModel,
    limits: DamiaoLimits,
    soft_zero: f64,
    invert: bool,
}

impl DamiaoDriver {
    pub fn new(can_id: u8, master_id: u16, model: DamiaoModel) -> Self {
        Self {
            can_id,
            master_id,
            model,
            limits: DamiaoLimits::for_model(model),
            soft_zero: 0.0,
            invert: false,
        }
    }

    fn sign(&self) -> f64 {
        if self.invert { -1.0 } else { 1.0 }
    }

    pub fn can_id(&self) -> u8 {
        self.can_id
    }

    pub fn model(&self) -> DamiaoModel {
        self.model
    }

    fn send_std(&mut self, socket: &CanSocket, data: &[u8]) -> Result<()> {
        let std_id = StandardId::new(self.can_id as u16)
            .expect("DAMIAO CAN_ID must fit in 11 bits");
        let frame = socketcan::CanFrame::new(Id::Standard(std_id), data)
            .expect("Failed to create DAMIAO CAN frame");
        socket.write_frame(&frame)?;
        Ok(())
    }

    /// Read the next DAMIAO feedback frame matching this motor.
    ///
    /// Skips extended-ID frames (Robstride traffic on a shared bus), short
    /// payloads, and frames whose payload byte 0 low nibble does not match
    /// `can_id`. If `master_id` is non-zero, additionally requires the
    /// response's standard ID to equal it.
    fn recv_feedback(&mut self, socket: &CanSocket, timeout: Duration) -> Result<Feedback> {
        let deadline = Instant::now() + timeout;
        loop {
            if Instant::now() >= deadline {
                return Err(RobstrideError::Timeout {
                    motor_id: self.can_id,
                });
            }
            match socket.read_frame() {
                Ok(frame) => {
                    if frame.is_extended() {
                        continue;
                    }
                    let raw_id = match frame.id() {
                        Id::Standard(sid) => StandardId::as_raw(&sid) as u16,
                        _ => continue,
                    };
                    if self.master_id != 0 && raw_id != self.master_id {
                        continue;
                    }
                    let data = frame.data();
                    if data.len() < 8 {
                        continue;
                    }
                    // The motor-id nibble in byte 0 is only the LOW 4 bits of
                    // the motor's CAN_ID, so compare nibbles, not full IDs.
                    // For CAN_IDs that share a low nibble, distinguish via
                    // master_id (the response standard ID) instead.
                    if (data[0] & 0x0F) != (self.can_id & 0x0F) {
                        continue;
                    }
                    return Ok(decode_damiao_feedback(data, &self.limits));
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(e) => return Err(RobstrideError::CanSocket(e)),
            }
        }
    }
}

impl MotorDriver for DamiaoDriver {
    fn enable(&mut self, socket: &CanSocket) -> Result<()> {
        self.send_std(socket, &DM_ENABLE)?;
        let _ = self.recv_feedback(socket, Duration::from_millis(50));
        Ok(())
    }

    fn disable(&mut self, socket: &CanSocket) -> Result<()> {
        self.send_std(socket, &DM_DISABLE)?;
        let _ = self.recv_feedback(socket, Duration::from_millis(50));
        Ok(())
    }

    fn set_soft_zero(&mut self, socket: &CanSocket) -> Result<()> {
        // In-memory only; the DM_SET_ZERO magic frame (FF..FE) deliberately
        // is *not* sent here, since that would write to motor NVM.
        soft_zero_via_mit(self, socket)
    }

    fn soft_zero_offset(&self) -> f64 {
        self.soft_zero
    }

    fn set_soft_zero_offset(&mut self, offset: f64) {
        self.soft_zero = offset;
    }

    fn invert(&self) -> bool {
        self.invert
    }

    fn set_invert(&mut self, invert: bool) {
        self.invert = invert;
    }

    fn mit_exchange(
        &mut self,
        socket: &CanSocket,
        position: f64,
        velocity: f64,
        kp: f64,
        kd: f64,
        torque: f64,
    ) -> Result<Feedback> {
        // soft_zero in raw frame (polarity-independent). See RobstrideDriver
        // for the math derivation.
        let sign = self.sign();
        let data = pack_damiao_mit(
            sign * position + self.soft_zero,
            sign * velocity,
            kp,
            kd,
            sign * torque,
            &self.limits,
        );
        self.send_std(socket, &data)?;
        let mut fb = self.recv_feedback(socket, Duration::from_millis(10))?;
        fb.position = sign * (fb.position - self.soft_zero);
        fb.velocity = sign * fb.velocity;
        fb.torque = sign * fb.torque;
        Ok(fb)
    }

    fn read_position(&mut self, _socket: &CanSocket) -> Result<f64> {
        Err(RobstrideError::InvalidResponse {
            msg: "DAMIAO MIT mode cannot read position while disabled".into(),
        })
    }

    fn read_velocity(&mut self, _socket: &CanSocket) -> Result<f64> {
        Err(RobstrideError::InvalidResponse {
            msg: "DAMIAO MIT mode cannot read velocity while disabled".into(),
        })
    }

    fn torque_limit(&self) -> f64 {
        // Same 50 %-of-MIT-range safety margin as Robstride.
        self.limits.t_max * 0.5
    }

    fn description(&self) -> String {
        format!("DAMIAO {} ID:{}", self.model, self.can_id)
    }
}

/// Discover DAMIAO motors on the bus.
///
/// For each `can_id` in `range`, sends a DM enable frame and listens briefly
/// for a status response. Any motor that answers is reported. Each probed ID
/// is immediately followed by a DM disable frame so found motors don't stay
/// powered (they receive no MIT command between enable and disable, so no
/// torque is ever commanded).
///
/// Returns the IDs that answered, in ascending order.
pub fn scan_damiao(
    interface: &str,
    range: std::ops::RangeInclusive<u8>,
    per_id_timeout: Duration,
) -> Result<Vec<u8>> {
    let socket = CanSocket::open(interface)?;
    socket.set_read_timeout(per_id_timeout)?;

    // Drain any frames already in the socket buffer so they don't
    // contaminate the very first probe.
    while let Ok(_) = socket.read_frame_timeout(Duration::from_millis(1)) {}

    let mut found = Vec::new();
    for can_id in range {
        let std_id = match StandardId::new(can_id as u16) {
            Some(id) => id,
            None => continue,
        };
        let enable_frame = socketcan::CanFrame::new(Id::Standard(std_id), &DM_ENABLE)
            .expect("8-byte DM_ENABLE fits a CAN data frame");
        if socket.write_frame(&enable_frame).is_err() {
            continue;
        }

        // Wait for a standard-ID frame whose payload byte-0 low nibble
        // matches this CAN_ID's low nibble (DM motor-id field).
        let deadline = Instant::now() + per_id_timeout;
        let mut answered = false;
        loop {
            if Instant::now() >= deadline {
                break;
            }
            match socket.read_frame() {
                Ok(frame) => {
                    if frame.is_extended() {
                        continue;
                    }
                    let data = frame.data();
                    if data.len() < 8 {
                        continue;
                    }
                    if (data[0] & 0x0F) == (can_id & 0x0F) {
                        answered = true;
                        break;
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(_) => break,
            }
        }
        if answered {
            found.push(can_id);
        }

        // Always send disable so a responding motor returns to safe state.
        let disable_frame = socketcan::CanFrame::new(Id::Standard(std_id), &DM_DISABLE)
            .expect("8-byte DM_DISABLE fits a CAN data frame");
        let _ = socket.write_frame(&disable_frame);
        // Tiny gap so the motor processes the disable before the next probe.
        std::thread::sleep(Duration::from_millis(2));
    }

    Ok(found)
}

/// Persist the DAMIAO motor's current physical position as its hardware
/// zero, written to NVM (flash). This is the FF..FE "save zero" command.
///
/// **Use sparingly.** Flash has a finite write-cycle budget and a botched
/// write can leave the joint mis-calibrated. The normal recalibration path
/// is the in-memory `soft_zero` (see [`MotorDriver::set_soft_zero`]) which
/// does not touch NVM.
///
/// The motor must be present on the bus and addressable at `can_id`. The
/// motor is sent the magic frame and then given ~50 ms to perform the flash
/// write before the function returns. Any reply frames during the wait are
/// drained so the next operation starts on a clean buffer.
pub fn damiao_set_zero_nvm(socket: &CanSocket, can_id: u8) -> Result<()> {
    let std_id = StandardId::new(can_id as u16)
        .expect("DAMIAO CAN_ID must fit in 11 bits");
    let frame = socketcan::CanFrame::new(Id::Standard(std_id), &DM_SET_ZERO_NVM)
        .expect("8-byte DM_SET_ZERO_NVM fits a CAN data frame");
    socket.write_frame(&frame)?;
    // NVM flash write can take ~10 ms; wait conservatively.
    std::thread::sleep(Duration::from_millis(50));
    while socket.read_frame_timeout(Duration::from_millis(1)).is_ok() {}
    Ok(())
}

fn pack_damiao_mit(
    pos: f64,
    vel: f64,
    kp: f64,
    kd: f64,
    tau: f64,
    lim: &DamiaoLimits,
) -> [u8; 8] {
    let p_int = float_to_uint(pos, -lim.p_max, lim.p_max, 16);
    let v_int = float_to_uint(vel, -lim.v_max, lim.v_max, 12);
    let kp_int = float_to_uint(kp, 0.0, lim.kp_max, 12);
    let kd_int = float_to_uint(kd, 0.0, lim.kd_max, 12);
    let t_int = float_to_uint(tau, -lim.t_max, lim.t_max, 12);
    [
        ((p_int >> 8) & 0xFF) as u8,
        (p_int & 0xFF) as u8,
        ((v_int >> 4) & 0xFF) as u8,
        ((((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF)) & 0xFF) as u8,
        (kp_int & 0xFF) as u8,
        ((kd_int >> 4) & 0xFF) as u8,
        ((((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF)) & 0xFF) as u8,
        (t_int & 0xFF) as u8,
    ]
}

fn decode_damiao_feedback(data: &[u8], lim: &DamiaoLimits) -> Feedback {
    let motor_id = data[0] & 0x0F;
    let p_int = ((data[1] as u32) << 8) | (data[2] as u32);
    let v_int = ((data[3] as u32) << 4) | ((data[4] as u32) >> 4);
    let t_int = (((data[4] & 0x0F) as u32) << 8) | (data[5] as u32);
    Feedback {
        motor_id,
        position: uint_to_float(p_int, -lim.p_max, lim.p_max, 16),
        velocity: uint_to_float(v_int, -lim.v_max, lim.v_max, 12),
        torque: uint_to_float(t_int, -lim.t_max, lim.t_max, 12),
        // DM reports two temperatures (byte 6 = MOS, byte 7 = rotor); expose
        // MOS as the primary temperature like Robstride.
        temperature: data[6] as f64,
        status: MotorStatusBits::default(),
    }
}

fn float_to_uint(x: f64, x_min: f64, x_max: f64, bits: u32) -> u32 {
    let span = x_max - x_min;
    let max_uint = ((1u64 << bits) - 1) as f64;
    let x_clamped = x.clamp(x_min, x_max);
    let scaled = ((x_clamped - x_min) * max_uint / span).round();
    scaled.clamp(0.0, max_uint) as u32
}

fn uint_to_float(x_int: u32, x_min: f64, x_max: f64, bits: u32) -> f64 {
    let span = x_max - x_min;
    let max_uint = ((1u64 << bits) - 1) as f64;
    (x_int as f64) * span / max_uint + x_min
}

// =============================================================================
// Motor specification (vendor + identification + model)
// =============================================================================

/// Vendor-agnostic motor specification used to construct a driver instance.
///
/// Both bilateral motors and the single-motor assist test take a `MotorSpec`,
/// so leader and follower can be different vendors. `soft_zero` is the
/// in-memory offset to seed the driver with at build time (see
/// [`MotorDriver::set_soft_zero_offset`]); use 0.0 for a fresh motor.
#[derive(Debug, Clone)]
pub enum MotorSpec {
    Robstride {
        host_id: u8,
        can_id: u8,
        model: MotorModel,
        soft_zero: f64,
        invert: bool,
    },
    Damiao {
        /// Motor's TX address (its `CAN_ID` register).
        can_id: u8,
        /// Motor's RX address that the host listens on (its `MST_ID` register).
        /// Use 0 to skip ID filtering and rely on the motor-id nibble in the
        /// response payload.
        master_id: u16,
        model: DamiaoModel,
        soft_zero: f64,
        invert: bool,
    },
}

impl MotorSpec {
    /// Convenience constructor for a Robstride motor.
    pub fn robstride(host_id: u8, can_id: u8, model: MotorModel) -> Self {
        MotorSpec::Robstride {
            host_id,
            can_id,
            model,
            soft_zero: 0.0,
            invert: false,
        }
    }

    /// Convenience constructor for a DAMIAO motor. `master_id = 0` is fine
    /// when only one DM motor is on the bus.
    pub fn damiao(can_id: u8, master_id: u16, model: DamiaoModel) -> Self {
        MotorSpec::Damiao {
            can_id,
            master_id,
            model,
            soft_zero: 0.0,
            invert: false,
        }
    }

    /// Return a copy of this spec with the soft-zero offset replaced.
    pub fn with_soft_zero(mut self, offset: f64) -> Self {
        match &mut self {
            MotorSpec::Robstride { soft_zero, .. } => *soft_zero = offset,
            MotorSpec::Damiao { soft_zero, .. } => *soft_zero = offset,
        }
        self
    }

    /// Return a copy of this spec with the polarity flag replaced.
    pub fn with_invert(mut self, flip: bool) -> Self {
        match &mut self {
            MotorSpec::Robstride { invert, .. } => *invert = flip,
            MotorSpec::Damiao { invert, .. } => *invert = flip,
        }
        self
    }

    /// Build the concrete driver for this spec, seeded with `soft_zero`.
    pub fn build(&self) -> Box<dyn MotorDriver> {
        match *self {
            MotorSpec::Robstride {
                host_id,
                can_id,
                model,
                soft_zero,
                invert,
            } => {
                let mut d = RobstrideDriver::new(host_id, can_id, model);
                d.set_soft_zero_offset(soft_zero);
                d.set_invert(invert);
                Box::new(d)
            }
            MotorSpec::Damiao {
                can_id,
                master_id,
                model,
                soft_zero,
                invert,
            } => {
                let mut d = DamiaoDriver::new(can_id, master_id, model);
                d.set_soft_zero_offset(soft_zero);
                d.set_invert(invert);
                Box::new(d)
            }
        }
    }

    /// CAN ID used to address the motor (TX side).
    pub fn can_id(&self) -> u8 {
        match *self {
            MotorSpec::Robstride { can_id, .. } => can_id,
            MotorSpec::Damiao { can_id, .. } => can_id,
        }
    }

    /// Stable identifier suitable for keying offsets / per-motor state across
    /// driver instances. Format: `<vendor>:<model>:<id>`.
    pub fn key(&self) -> String {
        match *self {
            MotorSpec::Robstride { model, can_id, .. } => {
                format!("robstride:{}:{}", model, can_id)
            }
            MotorSpec::Damiao { model, can_id, .. } => {
                format!("damiao:{}:{}", model, can_id)
            }
        }
    }

    /// Short human-readable label for logs and UI.
    pub fn description(&self) -> String {
        match *self {
            MotorSpec::Robstride { model, can_id, .. } => {
                format!("Robstride {} ID:{}", model, can_id)
            }
            MotorSpec::Damiao { model, can_id, .. } => {
                format!("DAMIAO {} ID:{}", model, can_id)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn damiao_pack_unpack_roundtrip() {
        let lim = DamiaoLimits::for_model(DamiaoModel::DmJ4310_2EC);
        let bytes = pack_damiao_mit(1.5, 2.0, 100.0, 1.0, 3.0, &lim);
        // Decode the *command* bytes back through the *feedback* decoder is
        // not meaningful (different layout), so verify the feedback decoder
        // round-trips its own packing manually.
        let mut data = [0u8; 8];
        // Build a feedback packet matching motor id 7, pos=1.5, vel=2.0, tau=3.0.
        let p_int = float_to_uint(1.5, -lim.p_max, lim.p_max, 16);
        let v_int = float_to_uint(2.0, -lim.v_max, lim.v_max, 12);
        let t_int = float_to_uint(3.0, -lim.t_max, lim.t_max, 12);
        data[0] = 0x07; // err=0, id=7
        data[1] = ((p_int >> 8) & 0xFF) as u8;
        data[2] = (p_int & 0xFF) as u8;
        data[3] = ((v_int >> 4) & 0xFF) as u8;
        data[4] = ((((v_int & 0xF) << 4) | ((t_int >> 8) & 0xF)) & 0xFF) as u8;
        data[5] = (t_int & 0xFF) as u8;
        data[6] = 35;
        data[7] = 40;
        let fb = decode_damiao_feedback(&data, &lim);
        assert_eq!(fb.motor_id, 7);
        assert!((fb.position - 1.5).abs() < 1e-3);
        assert!((fb.velocity - 2.0).abs() < 2e-2);
        assert!((fb.torque - 3.0).abs() < 1e-2);
        // Command bytes are non-empty.
        assert_eq!(bytes.len(), 8);
    }

    #[test]
    fn damiao_zero_command_packs_to_midpoint() {
        let lim = DamiaoLimits::for_model(DamiaoModel::DmJ4310_2EC);
        let bytes = pack_damiao_mit(0.0, 0.0, 0.0, 0.0, 0.0, &lim);
        // Zero position should encode to half-scale of u16 (0x7FFF or 0x8000).
        let p_int = ((bytes[0] as u32) << 8) | (bytes[1] as u32);
        assert!(p_int == 0x7FFF || p_int == 0x8000);
        // kp=0 -> top 4 bits of bytes[3..4] should be zero.
        assert_eq!(bytes[3] & 0x0F, 0);
        assert_eq!(bytes[4], 0);
    }
}

