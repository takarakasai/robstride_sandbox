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

use socketcan::{CanSocket, EmbeddedFrame, ExtendedId, Id, Socket};

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
pub trait MotorDriver: Send {
    /// Enable the motor (allow torque output).
    fn enable(&self, socket: &CanSocket) -> Result<()>;

    /// Disable the motor (free-spin / zero output).
    fn disable(&self, socket: &CanSocket) -> Result<()>;

    /// Send one MIT-mode command frame and return the resulting feedback.
    ///
    /// `position` [rad], `velocity` [rad/s], `kp` [Nm/rad], `kd` [Nm·s/rad],
    /// `torque` [Nm] — all in physical units; the driver scales them.
    fn mit_exchange(
        &self,
        socket: &CanSocket,
        position: f64,
        velocity: f64,
        kp: f64,
        kd: f64,
        torque: f64,
    ) -> Result<Feedback>;

    /// Read mechanical position [rad]. Must work while the motor is disabled
    /// (used by the OnDemand bilateral method to keep the leader free).
    fn read_position(&self, socket: &CanSocket) -> Result<f64>;

    /// Read mechanical velocity [rad/s]. Same disabled-read requirement as
    /// [`MotorDriver::read_position`].
    fn read_velocity(&self, socket: &CanSocket) -> Result<f64>;

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
}

impl RobstrideDriver {
    pub fn new(host_id: u8, motor_id: u8, model: MotorModel) -> Self {
        Self {
            host_id,
            motor_id,
            model,
            scales: MitScales::for_model(model),
        }
    }

    pub fn motor_id(&self) -> u8 {
        self.motor_id
    }

    pub fn model(&self) -> MotorModel {
        self.model
    }

    fn read_param(&self, socket: &CanSocket, param: ParamIndex) -> Result<f32> {
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
    fn enable(&self, socket: &CanSocket) -> Result<()> {
        let (can_id, data) = build_enable_frame(self.host_id, self.motor_id);
        send_can(socket, can_id, &data)?;
        // Consume the response frame if it arrives; ignore timeout.
        let _ = recv_can(socket, Duration::from_millis(50));
        Ok(())
    }

    fn disable(&self, socket: &CanSocket) -> Result<()> {
        let (can_id, data) = build_disable_frame(self.host_id, self.motor_id);
        send_can(socket, can_id, &data)?;
        let _ = recv_can(socket, Duration::from_millis(50));
        Ok(())
    }

    fn mit_exchange(
        &self,
        socket: &CanSocket,
        position: f64,
        velocity: f64,
        kp: f64,
        kd: f64,
        torque: f64,
    ) -> Result<Feedback> {
        let (can_id, data) = build_mit_frame(
            self.host_id,
            self.motor_id,
            &self.scales,
            position,
            velocity,
            kp,
            kd,
            torque,
        );
        send_can(socket, can_id, &data)?;

        // Read response; skip echoes / unrelated motor frames.
        let deadline = Instant::now() + Duration::from_millis(10);
        loop {
            let timeout = deadline
                .duration_since(Instant::now())
                .max(Duration::from_millis(1));
            let (ct, extra, dev, rdata) = recv_can(socket, timeout)?;
            if ct == CommType::OperationStatus as u8 {
                let raw = build_can_id_raw(ct, extra, dev);
                if let Some(fb) = parse_status_frame(raw, &rdata, &self.scales) {
                    if fb.motor_id == self.motor_id {
                        return Ok(fb);
                    }
                }
            }
        }
    }

    fn read_position(&self, socket: &CanSocket) -> Result<f64> {
        Ok(self.read_param(socket, ParamIndex::MechPos)? as f64)
    }

    fn read_velocity(&self, socket: &CanSocket) -> Result<f64> {
        Ok(self.read_param(socket, ParamIndex::MechVel)? as f64)
    }

    fn torque_limit(&self) -> f64 {
        self.scales.torque * 0.5
    }

    fn description(&self) -> String {
        format!("Robstride {} ID:{}", self.model, self.motor_id)
    }
}
