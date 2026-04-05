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
    /// Coulomb friction estimate [Nm] (constant friction opposing motion)
    pub coulomb_friction: f64,
    /// Viscous friction coefficient [Nm·s/rad] (friction proportional to velocity)
    pub viscous_friction: f64,
    /// Leader inertia compensation ratio [0.0 - 1.0]
    /// 0.0 = no compensation, 1.0 = full cancellation of motor inertia
    pub inertia_comp: f64,
    /// LPF cutoff for acceleration estimation [rad/s] (for inertia comp)
    pub accel_cutoff: f64,
    /// Leader motor-internal kd for velocity assist [Nm·s/rad]
    /// Uses the motor's built-in MIT kd (runs at motor rate ~10kHz)
    /// to provide active backdrive assist, overcoming gearbox friction/inertia.
    /// 0.0 = disabled, typical: 0.1-0.5
    pub assist_kd: f64,
    /// Velocity reference lookahead factor for leader assist.
    /// vel_ref = measured_vel * vel_ahead.
    /// 1.5-3.0 typical. Higher = more aggressive assist.
    pub vel_ahead: f64,
}

impl Default for BilateralGains {
    fn default() -> Self {
        BilateralGains {
            kp: 5.0,
            kd: 0.3,
            force_scale: 0.5,
            inertia: 0.005,
            dob_cutoff: 100.0,
            coulomb_friction: 0.05,
            viscous_friction: 0.01,
            inertia_comp: 0.0,
            accel_cutoff: 50.0,
            assist_kd: 0.0,
            vel_ahead: 2.0,
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
    /// Leader friction compensation torque [Nm]
    pub leader_friction_comp: f64,
    /// Follower friction compensation torque [Nm]
    pub follower_friction_comp: f64,
    /// Leader inertia compensation torque [Nm]
    pub leader_inertia_comp: f64,
    /// Leader velocity assist (motor-internal kd contribution) [Nm]
    pub leader_vel_assist: f64,
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

/// Compute friction compensation feedforward torque.
///
/// Returns a torque in the direction of motion to overcome internal friction:
///   τ_comp = coulomb·sign(ω) + viscous·ω
///
/// A small dead zone (|ω| < 0.01) avoids sign chatter at zero velocity.
fn friction_compensation(velocity: f64, coulomb: f64, viscous: f64) -> f64 {
    // Deadband 0.05 rad/s to avoid step torque from velocity noise
    let sign = if velocity.abs() < 0.05 {
        0.0
    } else {
        velocity.signum()
    };
    coulomb * sign + viscous * velocity
}

/// Soft-start ramp: linearly ramps from 0 to 1 over `ramp_secs`.
const SOFT_START_SECS: f64 = 2.0;

fn soft_start_gain(elapsed_secs: f64) -> f64 {
    (elapsed_secs / SOFT_START_SECS).clamp(0.0, 1.0)
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
    let coulomb = config.gains.coulomb_friction;
    let viscous = config.gains.viscous_friction;
    let inertia_comp_ratio = config.gains.inertia_comp.clamp(0.0, 1.0);
    let j_comp = config.gains.inertia * inertia_comp_ratio;
    let loop_period = Duration::from_micros(config.loop_period_us);
    let mut cycle: u64 = 0;
    let mut loop_start = Instant::now();
    let mut hz_accum = 0.0;
    let mut hz_count = 0u32;

    // Leader acceleration estimator: LPF on dω/dt
    let mut leader_prev_vel = prev_l_vel;
    let mut accel_lpf = LowPassFilter::new(config.gains.accel_cutoff);

    // Clamp torque to motor limits
    let torque_limit = scales.torque * 0.5; // leave 50% margin for safety
    let start_time = Instant::now();

    loop {
        if stop.load(Ordering::Relaxed) {
            break;
        }

        let iter_start = Instant::now();
        let ramp = soft_start_gain(start_time.elapsed().as_secs_f64());
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
                let tau_ext_l = dob_leader.update(0.0, prev_l_vel, dt);
                let tau_ext_f = dob_follower.update(0.0, prev_f_vel, dt);

                let pos_err = prev_l_pos - prev_f_pos;
                let vel_err = prev_l_vel - prev_f_vel;
                let tau_diff = kp * pos_err + kd * vel_err;

                let tau_l = -tau_diff + tau_ext_f;
                let tau_f = tau_diff + tau_ext_l;

                (tau_l, tau_f)
            }
        };

        // Friction compensation feedforward for each motor.
        // This cancels internal motor friction so it does not propagate
        // through the virtual coupling as a phantom force.
        //   τ_comp = coulomb·sign(ω) + viscous·ω
        // Applied in the direction of existing velocity to overcome friction.
        let friction_comp_l = friction_compensation(prev_l_vel, coulomb, viscous);
        let friction_comp_f = friction_compensation(prev_f_vel, coulomb, viscous);

        // Leader inertia compensation: τ = -J_comp · α_filtered
        // This makes the leader feel lighter by cancelling its own inertia.
        let raw_accel = if dt > 0.0 {
            (prev_l_vel - leader_prev_vel) / dt
        } else {
            0.0
        };
        leader_prev_vel = prev_l_vel;
        let filtered_accel = accel_lpf.update(raw_accel, dt);
        let inertia_comp_torque = -j_comp * filtered_accel;

        // Apply soft-start ramp to all compensation torques
        let tau_leader_total = tau_leader + (friction_comp_l + inertia_comp_torque) * ramp;
        let tau_follower_total = tau_follower + friction_comp_f * ramp;

        // Clamp
        let tau_leader_clamped = tau_leader_total.clamp(-torque_limit, torque_limit);
        let tau_follower_clamped = tau_follower_total.clamp(-torque_limit, torque_limit);

        // =====================================================================
        // Leader: use motor-internal kd for velocity assist
        // =====================================================================
        // Instead of kp=0, kd=0, τ_ff=everything:
        //   kp=0, kd=assist_kd, vel_ref=vel*vel_ahead, τ_ff=coupling+friction
        //
        // Motor internally computes:
        //   τ_motor = kd*(vel_ref - vel_actual) + τ_ff
        //           ≈ assist_kd*(vel_ahead-1)*vel + τ_ff
        //
        // This "negative damping" runs at motor's internal rate (~10kHz),
        // providing much faster assist than CAN-rate feedforward.
        let (mit_kd_leader, mit_vel_leader) = if config.gains.assist_kd > 0.0 {
            // vel_ref slightly ahead of current velocity → motor assists motion
            // Ramp up kd for safety during soft-start
            let vel_ref = prev_l_vel * config.gains.vel_ahead;
            (config.gains.assist_kd * ramp, vel_ref)
        } else {
            (0.0, 0.0)
        };
        let leader_vel_assist_est = if config.gains.assist_kd > 0.0 {
            config.gains.assist_kd * ramp * (config.gains.vel_ahead - 1.0) * prev_l_vel
        } else {
            0.0
        };

        // Send MIT commands
        let fb_l = match mit_exchange(
            &socket, host, lid, &scales,
            0.0, mit_vel_leader, 0.0, mit_kd_leader, tau_leader_clamped,
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
                t.leader_friction_comp = friction_comp_l;
                t.follower_friction_comp = friction_comp_f;
                t.leader_inertia_comp = inertia_comp_torque;
                t.leader_vel_assist = leader_vel_assist_est;
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

// =============================================================================
// Single-motor Assist Test
// =============================================================================

/// Configuration for the single-motor assist test.
#[derive(Debug, Clone)]
pub struct AssistTestConfig {
    pub interface: String,
    pub host_id: u8,
    pub motor_id: u8,
    pub model: MotorModel,
    /// Motor-internal kd for velocity assist
    pub assist_kd: f64,
    /// Velocity reference lookahead factor
    pub vel_ahead: f64,
    /// Coulomb friction compensation [Nm]
    pub coulomb_friction: f64,
    /// Viscous friction compensation [Nm·s/rad]
    pub viscous_friction: f64,
    /// Motor inertia [kg·m²] (for CAN-side inertia comp)
    pub inertia: f64,
    /// Inertia compensation ratio [0-1]
    pub inertia_comp: f64,
    /// Acceleration LPF cutoff [rad/s]
    pub accel_cutoff: f64,
    /// Target loop period [µs]
    pub loop_period_us: u64,
}

impl Default for AssistTestConfig {
    fn default() -> Self {
        AssistTestConfig {
            interface: "can0".to_string(),
            host_id: 0xFD,
            motor_id: 10,
            model: MotorModel::Rs05,
            assist_kd: 0.0,
            vel_ahead: 2.0,
            coulomb_friction: 0.0,
            viscous_friction: 0.0,
            inertia: 0.005,
            inertia_comp: 0.0,
            accel_cutoff: 50.0,
            loop_period_us: 2000,
        }
    }
}

/// Launch the single-motor assist test loop in a background thread.
///
/// Returns (telemetry_handle, stop_flag).
/// Telemetry uses the same BilateralTelemetry struct (follower fields stay zero).
pub fn launch_assist_test(
    config: AssistTestConfig,
) -> Result<(SharedTelemetry, StopFlag)> {
    let telemetry = Arc::new(Mutex::new(BilateralTelemetry {
        method: None, // indicates assist-test mode
        ..Default::default()
    }));
    let stop = Arc::new(AtomicBool::new(false));

    let telem = Arc::clone(&telemetry);
    let stop_flag = Arc::clone(&stop);

    std::thread::spawn(move || {
        if let Err(e) = run_assist_test_loop(&config, &telem, &stop_flag) {
            if let Ok(mut t) = telem.lock() {
                t.last_error = Some(format!("Assist test error: {}", e));
            }
        }
    });

    Ok((telemetry, stop))
}

fn run_assist_test_loop(
    config: &AssistTestConfig,
    telemetry: &SharedTelemetry,
    stop: &StopFlag,
) -> Result<()> {
    let socket = CanSocket::open(&config.interface)?;
    socket.set_read_timeout(Duration::from_millis(10))?;

    let scales = MitScales::for_model(config.model);
    let host = config.host_id;
    let mid = config.motor_id;

    // Enable motor
    can_enable(&socket, host, mid)?;
    std::thread::sleep(Duration::from_millis(20));

    // Initial status read
    let fb = mit_exchange(&socket, host, mid, &scales, 0.0, 0.0, 0.0, 0.0, 0.0)?;
    let mut prev_vel = fb.velocity;
    let mut prev_prev_vel = prev_vel;

    let coulomb = config.coulomb_friction;
    let viscous = config.viscous_friction;
    let j_comp = config.inertia * config.inertia_comp.clamp(0.0, 1.0);
    let loop_period = Duration::from_micros(config.loop_period_us);

    let mut accel_lpf = LowPassFilter::new(config.accel_cutoff);
    let mut cycle: u64 = 0;
    let mut loop_start = Instant::now();
    let mut hz_accum = 0.0;
    let mut hz_count = 0u32;

    let torque_limit = scales.torque * 0.5; // leave 50% margin for safety
    let start_time = Instant::now();

    loop {
        if stop.load(Ordering::Relaxed) {
            break;
        }

        let iter_start = Instant::now();
        let ramp = soft_start_gain(start_time.elapsed().as_secs_f64());
        let dt = if cycle == 0 {
            loop_period.as_secs_f64()
        } else {
            iter_start.duration_since(loop_start).as_secs_f64().max(0.0001)
        };
        loop_start = iter_start;

        // Friction compensation
        let friction_comp = friction_compensation(prev_vel, coulomb, viscous);

        // CAN-side inertia compensation
        let raw_accel = if dt > 0.0 {
            (prev_vel - prev_prev_vel) / dt
        } else {
            0.0
        };
        prev_prev_vel = prev_vel;
        let filtered_accel = accel_lpf.update(raw_accel, dt);
        let inertia_comp_torque = -j_comp * filtered_accel;

        // Total CAN-side feedforward torque (with soft-start ramp)
        let tau_ff = ((friction_comp + inertia_comp_torque) * ramp).clamp(-torque_limit, torque_limit);

        // Motor-internal velocity assist (with soft-start ramp)
        let (mit_kd, mit_vel) = if config.assist_kd > 0.0 {
            (config.assist_kd * ramp, prev_vel * config.vel_ahead)
        } else {
            (0.0, 0.0)
        };
        let vel_assist_est = if config.assist_kd > 0.0 {
            config.assist_kd * ramp * (config.vel_ahead - 1.0) * prev_vel
        } else {
            0.0
        };

        // Send MIT command
        let fb = match mit_exchange(
            &socket, host, mid, &scales,
            0.0, mit_vel, 0.0, mit_kd, tau_ff,
        ) {
            Ok(fb) => fb,
            Err(_e) => {
                cycle += 1;
                continue;
            }
        };

        prev_vel = fb.velocity;

        // Loop frequency
        let elapsed = iter_start.elapsed().as_secs_f64();
        if elapsed > 0.0 {
            hz_accum += 1.0 / elapsed;
            hz_count += 1;
        }

        // Update telemetry
        if cycle % 10 == 0 {
            if let Ok(mut t) = telemetry.lock() {
                t.leader_pos = fb.position;
                t.leader_vel = fb.velocity;
                t.leader_torque_cmd = tau_ff;
                t.leader_friction_comp = friction_comp;
                t.leader_inertia_comp = inertia_comp_torque;
                t.leader_vel_assist = vel_assist_est;
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

        let work_time = iter_start.elapsed();
        if work_time < loop_period {
            std::thread::sleep(loop_period - work_time);
        }
    }

    let _ = can_disable(&socket, host, mid);
    Ok(())
}
