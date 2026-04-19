//! Bilateral control implementations for dual-motor haptic teleoperation.
//!
//! Four methods are provided:
//!
//! 1. **Position Mirroring** – Follower tracks leader position. No force feedback.
//! 2. **Force-Reflecting** – Follower tracks leader position; leader feels follower reaction force.
//! 3. **Virtual Coupling** – Symmetric virtual spring-damper between the two motors.
//! 4. **Mode-Space (4ch)** – Differential/common mode decomposition with observers.
//!
//! All methods run a real-time control loop that communicates with two motors
//! over CAN bus using MIT mode.

use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use std::time::{Duration, Instant};

use socketcan::{CanSocket, EmbeddedFrame, ExtendedId, Id, Socket};

use crate::error::{Result, RobstrideError};
use crate::protocol::*;

// =============================================================================
// Types
// =============================================================================

/// Which bilateral control method to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BilateralMethod {
    /// 1: Follower mirrors leader position. Leader runs free (low damping).
    PositionMirroring,
    /// 2: Follower tracks leader; leader receives reflected force from follower.
    ForceReflecting,
    /// 3: Virtual spring-damper coupling between the two motors.
    VirtualCoupling,
    /// 4: Mode-space (4ch) bilateral with observer-based force estimation.
    ModeSpace,
}

impl BilateralMethod {
    pub const ALL: [BilateralMethod; 4] = [
        BilateralMethod::PositionMirroring,
        BilateralMethod::ForceReflecting,
        BilateralMethod::VirtualCoupling,
        BilateralMethod::ModeSpace,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            BilateralMethod::PositionMirroring => "Position Mirroring",
            BilateralMethod::ForceReflecting => "Force Reflecting",
            BilateralMethod::VirtualCoupling => "Virtual Coupling",
            BilateralMethod::ModeSpace => "Mode Space (4ch)",
        }
    }

    pub fn short(&self) -> &'static str {
        match self {
            BilateralMethod::PositionMirroring => "pos",
            BilateralMethod::ForceReflecting => "force",
            BilateralMethod::VirtualCoupling => "coupling",
            BilateralMethod::ModeSpace => "mode",
        }
    }

    /// Parse from short name.
    pub fn from_short(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "pos" | "position" | "1" => Some(Self::PositionMirroring),
            "force" | "2" => Some(Self::ForceReflecting),
            "coupling" | "virtual" | "3" => Some(Self::VirtualCoupling),
            "mode" | "4ch" | "4" => Some(Self::ModeSpace),
            _ => None,
        }
    }
}

/// Gains for bilateral control.
#[derive(Debug, Clone, Copy)]
pub struct BilateralGains {
    /// Position gain (virtual spring stiffness) [Nm/rad]
    pub kp: f64,
    /// Velocity gain (virtual damper) [Nm·s/rad]
    pub kd: f64,
    /// Force reflection scaling (for Force-Reflecting method) [0..1]
    pub force_scale: f64,
    /// Estimated motor inertia [kg·m²] (for Mode-Space DOB)
    pub inertia: f64,
    /// DOB cutoff frequency [rad/s] (for Mode-Space)
    pub dob_cutoff: f64,
}

impl Default for BilateralGains {
    fn default() -> Self {
        BilateralGains {
            kp: 5.0,
            kd: 0.3,
            force_scale: 0.5,
            inertia: 0.005,
            dob_cutoff: 100.0,
        }
    }
}

/// Real-time telemetry snapshot shared with the UI.
#[derive(Debug, Clone, Default)]
pub struct BilateralTelemetry {
    /// Leader measured position [rad]
    pub leader_pos: f64,
    /// Leader measured velocity [rad/s]
    pub leader_vel: f64,
    /// Leader commanded torque [Nm]
    pub leader_torque_cmd: f64,
    /// Follower measured position [rad]
    pub follower_pos: f64,
    /// Follower measured velocity [rad/s]
    pub follower_vel: f64,
    /// Follower commanded torque [Nm]
    pub follower_torque_cmd: f64,
    /// Control loop frequency [Hz]
    pub loop_hz: f64,
    /// Position error θ_leader - θ_follower [rad]
    pub position_error: f64,
    /// Number of control cycles executed
    pub cycle_count: u64,
    /// Active method
    pub method: Option<BilateralMethod>,
    /// Last error message (if any)
    pub last_error: Option<String>,
}

/// Shared state for communicating with the bilateral control thread.
pub type SharedTelemetry = Arc<Mutex<BilateralTelemetry>>;

/// Flag to request the control loop to stop.
pub type StopFlag = Arc<AtomicBool>;

// =============================================================================
// Low-level CAN helpers (thread-local, no Motor struct dependency)
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

/// Send MIT command and receive feedback for one motor.
fn mit_exchange(
    socket: &CanSocket,
    host_id: u8,
    motor_id: u8,
    scales: &MitScales,
    position: f64,
    velocity: f64,
    kp: f64,
    kd: f64,
    torque: f64,
) -> Result<MotorFeedback> {
    let (can_id, data) = build_mit_frame(host_id, motor_id, scales, position, velocity, kp, kd, torque);
    send_can(socket, can_id, &data)?;

    // Read response, skip echo / unrelated frames
    let deadline = Instant::now() + Duration::from_millis(10);
    loop {
        let (ct, extra, dev, rdata) = recv_can(socket, deadline.duration_since(Instant::now()).max(Duration::from_millis(1)))?;
        if ct == CommType::OperationStatus as u8 {
            let raw = build_can_id_raw(ct, extra, dev);
            if let Some(fb) = parse_status_frame(raw, &rdata, scales) {
                if fb.motor_id == motor_id {
                    return Ok(fb);
                }
            }
        }
    }
}

/// Enable a motor via CAN (does not use Motor struct).
fn can_enable(socket: &CanSocket, host_id: u8, motor_id: u8) -> Result<()> {
    let (can_id, data) = build_enable_frame(host_id, motor_id);
    send_can(socket, can_id, &data)?;
    // consume response
    let _ = recv_can(socket, Duration::from_millis(50));
    Ok(())
}

/// Disable a motor via CAN.
fn can_disable(socket: &CanSocket, host_id: u8, motor_id: u8) -> Result<()> {
    let (can_id, data) = build_disable_frame(host_id, motor_id);
    send_can(socket, can_id, &data)?;
    let _ = recv_can(socket, Duration::from_millis(50));
    Ok(())
}

/// Read current iq_filt from a motor.
#[allow(dead_code)]
fn can_read_current(socket: &CanSocket, host_id: u8, motor_id: u8) -> Result<f32> {
    let (can_id, data) = build_read_param_frame(host_id, motor_id, ParamIndex::IqFilt);
    send_can(socket, can_id, &data)?;
    let deadline = Instant::now() + Duration::from_millis(20);
    loop {
        let (_ct, _extra, _dev, rdata) = recv_can(socket, deadline.duration_since(Instant::now()).max(Duration::from_millis(1)))?;
        if let Some((_idx, val)) = parse_param_response(&rdata) {
            return Ok(val);
        }
    }
}

// =============================================================================
// First-order low-pass filter (for DOB)
// =============================================================================

#[derive(Debug, Clone)]
struct LowPassFilter {
    cutoff: f64,  // rad/s
    output: f64,
}

impl LowPassFilter {
    fn new(cutoff: f64) -> Self {
        LowPassFilter { cutoff, output: 0.0 }
    }

    /// Update with new input and time step dt [s]. Returns filtered output.
    fn update(&mut self, input: f64, dt: f64) -> f64 {
        // Tustin (bilinear) discretization: alpha = wc*dt / (2 + wc*dt)
        let alpha = (self.cutoff * dt) / (2.0 + self.cutoff * dt);
        self.output = self.output + alpha * (input - self.output);
        self.output
    }

    #[allow(dead_code)]
    fn reset(&mut self) {
        self.output = 0.0;
    }
}

/// Disturbance observer for one motor axis.
#[derive(Debug, Clone)]
struct DisturbanceObserver {
    inertia: f64,       // J [kg·m²]
    lpf: LowPassFilter, // low-pass filter for disturbance estimation
    prev_vel: f64,
}

impl DisturbanceObserver {
    fn new(inertia: f64, cutoff: f64) -> Self {
        DisturbanceObserver {
            inertia,
            lpf: LowPassFilter::new(cutoff),
            prev_vel: 0.0,
        }
    }

    /// Estimate external torque.
    ///
    /// `torque_cmd` - commanded torque
    /// `velocity` - measured velocity
    /// `dt` - time step [s]
    ///
    /// Returns estimated disturbance torque (≈ external torque if friction is small).
    fn update(&mut self, torque_cmd: f64, velocity: f64, dt: f64) -> f64 {
        if dt <= 0.0 {
            return 0.0;
        }
        // Disturbance = τ_cmd - J * dω/dt
        // We filter (τ_cmd - J * ω_dot) through LPF
        let accel = (velocity - self.prev_vel) / dt;
        self.prev_vel = velocity;
        let disturbance_raw = torque_cmd - self.inertia * accel;
        self.lpf.update(disturbance_raw, dt)
    }

    #[allow(dead_code)]
    fn reset(&mut self) {
        self.lpf.reset();
        self.prev_vel = 0.0;
    }
}

// =============================================================================
// Main bilateral control loop
// =============================================================================

/// Configuration for launching the bilateral control loop.
#[derive(Debug, Clone)]
pub struct BilateralConfig {
    pub interface: String,
    pub host_id: u8,
    pub leader_id: u8,
    pub follower_id: u8,
    pub model: MotorModel,
    pub method: BilateralMethod,
    pub gains: BilateralGains,
    /// Target loop period [µs]
    pub loop_period_us: u64,
}

impl Default for BilateralConfig {
    fn default() -> Self {
        BilateralConfig {
            interface: "can0".to_string(),
            host_id: 0xFD,
            leader_id: 10,
            follower_id: 1,
            model: MotorModel::Rs05,
            method: BilateralMethod::VirtualCoupling,
            gains: BilateralGains::default(),
            loop_period_us: 2000, // 500 Hz target
        }
    }
}

/// Launch the bilateral control loop in a background thread.
///
/// Returns (telemetry_handle, stop_flag) that let the caller monitor and stop the loop.
pub fn launch_bilateral(
    config: BilateralConfig,
) -> Result<(SharedTelemetry, StopFlag)> {
    let telemetry = Arc::new(Mutex::new(BilateralTelemetry {
        method: Some(config.method),
        ..Default::default()
    }));
    let stop = Arc::new(AtomicBool::new(false));

    let telem = Arc::clone(&telemetry);
    let stop_flag = Arc::clone(&stop);

    std::thread::spawn(move || {
        if let Err(e) = run_bilateral_loop(&config, &telem, &stop_flag) {
            if let Ok(mut t) = telem.lock() {
                t.last_error = Some(format!("Loop error: {}", e));
            }
        }
    });

    Ok((telemetry, stop))
}

/// The actual control loop (runs in a dedicated thread).
fn run_bilateral_loop(
    config: &BilateralConfig,
    telemetry: &SharedTelemetry,
    stop: &StopFlag,
) -> Result<()> {
    let socket = CanSocket::open(&config.interface)?;
    socket.set_read_timeout(Duration::from_millis(10))?;

    let scales = MitScales::for_model(config.model);
    let host = config.host_id;
    let lid = config.leader_id;
    let fid = config.follower_id;

    // Enable both motors
    can_enable(&socket, host, lid)?;
    std::thread::sleep(Duration::from_millis(20));
    can_enable(&socket, host, fid)?;
    std::thread::sleep(Duration::from_millis(20));

    // Initial status read (MIT zero command)
    let fb_l = mit_exchange(&socket, host, lid, &scales, 0.0, 0.0, 0.0, 0.0, 0.0)?;
    let fb_f = mit_exchange(&socket, host, fid, &scales, 0.0, 0.0, 0.0, 0.0, 0.0)?;

    let mut prev_l_pos = fb_l.position;
    let mut prev_f_pos = fb_f.position;
    let mut prev_l_vel = fb_l.velocity;
    let mut prev_f_vel = fb_f.velocity;

    // Force-Reflecting: for reading follower current
    let mut follower_torque_est: f64 = 0.0;

    // Mode-Space: DOBs
    let mut dob_leader = DisturbanceObserver::new(config.gains.inertia, config.gains.dob_cutoff);
    let mut dob_follower = DisturbanceObserver::new(config.gains.inertia, config.gains.dob_cutoff);

    let kp = config.gains.kp;
    let kd = config.gains.kd;
    let loop_period = Duration::from_micros(config.loop_period_us);
    let mut cycle: u64 = 0;
    let mut loop_start = Instant::now();
    let mut hz_accum = 0.0;
    let mut hz_count = 0u32;

    // Clamp torque to motor limits
    let torque_limit = scales.torque * 0.8; // leave 20% margin

    loop {
        if stop.load(Ordering::Relaxed) {
            break;
        }

        let iter_start = Instant::now();
        let dt = if cycle == 0 {
            loop_period.as_secs_f64()
        } else {
            iter_start.duration_since(loop_start).as_secs_f64().max(0.0001)
        };
        loop_start = iter_start;

        // Compute torques based on method
        let (tau_leader, tau_follower) = match config.method {
            BilateralMethod::PositionMirroring => {
                // Leader: free motion (low damping only)
                let tau_l = -0.05 * prev_l_vel; // very light damping
                // Follower: PD tracking of leader position
                let err = prev_l_pos - prev_f_pos;
                let derr = prev_l_vel - prev_f_vel;
                let tau_f = kp * err + kd * derr;
                (tau_l, tau_f)
            }

            BilateralMethod::ForceReflecting => {
                // Follower: PD tracking of leader position
                let err = prev_l_pos - prev_f_pos;
                let derr = prev_l_vel - prev_f_vel;
                let tau_f = kp * err + kd * derr;
                // Leader: reflected force from follower
                let tau_l = -config.gains.force_scale * follower_torque_est
                    - 0.05 * prev_l_vel;
                (tau_l, tau_f)
            }

            BilateralMethod::VirtualCoupling => {
                // Symmetric spring-damper
                let err = prev_l_pos - prev_f_pos;
                let derr = prev_l_vel - prev_f_vel;
                let coupling = kp * err + kd * derr;
                // Leader gets -coupling, follower gets +coupling
                (-coupling, coupling)
            }

            BilateralMethod::ModeSpace => {
                // 4-channel bilateral with DOB-based force estimation
                //
                // Differential mode: θ_L - θ_F → 0  (position tracking)
                // Common mode:  τ_ext_L + τ_ext_F → 0  (force transparency)
                //
                // τ_ext estimated by DOB: τ_ext = τ_cmd - J·α (filtered)

                let tau_ext_l = dob_leader.update(0.0, prev_l_vel, dt);
                let tau_ext_f = dob_follower.update(0.0, prev_f_vel, dt);

                // Position error (differential)
                let pos_err = prev_l_pos - prev_f_pos;
                let vel_err = prev_l_vel - prev_f_vel;

                // Differential mode controller
                let tau_diff = kp * pos_err + kd * vel_err;

                // Common mode controller → force transparency
                // τ_L + τ_F ≈ τ_ext_L + τ_ext_F should → 0
                // We add force compensation: each motor also reflects the other's force
                let tau_l = -tau_diff + tau_ext_f;
                let tau_f = tau_diff + tau_ext_l;

                (tau_l, tau_f)
            }
        };

        // Clamp
        let tau_leader_clamped = tau_leader.clamp(-torque_limit, torque_limit);
        let tau_follower_clamped = tau_follower.clamp(-torque_limit, torque_limit);

        // Send MIT commands (kp=0, kd=0, torque feedforward only)
        let fb_l = match mit_exchange(
            &socket, host, lid, &scales,
            0.0, 0.0, 0.0, 0.0, tau_leader_clamped,
        ) {
            Ok(fb) => fb,
            Err(_e) => {
                // On timeout, keep previous values
                cycle += 1;
                continue;
            }
        };

        let fb_f = match mit_exchange(
            &socket, host, fid, &scales,
            0.0, 0.0, 0.0, 0.0, tau_follower_clamped,
        ) {
            Ok(fb) => fb,
            Err(_e) => {
                cycle += 1;
                continue;
            }
        };

        prev_l_pos = fb_l.position;
        prev_l_vel = fb_l.velocity;
        prev_f_pos = fb_f.position;
        prev_f_vel = fb_f.velocity;

        // Force-Reflecting: estimate follower external torque from iq_filt
        if config.method == BilateralMethod::ForceReflecting {
            // Use torque feedback directly as estimate
            follower_torque_est = fb_f.torque;
        }

        // Mode-Space: update DOB with actual commanded torques
        if config.method == BilateralMethod::ModeSpace {
            dob_leader.update(tau_leader_clamped, fb_l.velocity, dt);
            dob_follower.update(tau_follower_clamped, fb_f.velocity, dt);
        }

        // Compute loop frequency
        let elapsed = iter_start.elapsed().as_secs_f64();
        if elapsed > 0.0 {
            hz_accum += 1.0 / elapsed;
            hz_count += 1;
        }

        // Update telemetry (not every cycle to reduce contention)
        if cycle % 10 == 0 {
            if let Ok(mut t) = telemetry.lock() {
                t.leader_pos = fb_l.position;
                t.leader_vel = fb_l.velocity;
                t.leader_torque_cmd = tau_leader_clamped;
                t.follower_pos = fb_f.position;
                t.follower_vel = fb_f.velocity;
                t.follower_torque_cmd = tau_follower_clamped;
                t.position_error = fb_l.position - fb_f.position;
                t.cycle_count = cycle;
                if hz_count > 0 {
                    t.loop_hz = hz_accum / hz_count as f64;
                    hz_accum = 0.0;
                    hz_count = 0;
                }
                t.last_error = None;
            }
        }

        cycle += 1;

        // Sleep to maintain target loop rate
        let work_time = iter_start.elapsed();
        if work_time < loop_period {
            std::thread::sleep(loop_period - work_time);
        }
    }

    // Disable both motors on exit
    let _ = can_disable(&socket, host, lid);
    let _ = can_disable(&socket, host, fid);

    Ok(())
}
