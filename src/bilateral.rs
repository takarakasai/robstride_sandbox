//! Bilateral control implementations for dual-motor haptic teleoperation.
//!
//! Six methods are provided:
//!
//! 1. **Position Mirroring** – Follower tracks leader position. No force feedback.
//! 2. **Force-Reflecting** – Follower tracks leader position; leader feels follower reaction force.
//! 3. **Virtual Coupling** – Symmetric virtual spring-damper between the two motors.
//! 4. **Mode-Space (4ch)** – Differential/common mode decomposition with observers.
//! 5. **On-Demand** – Leader stays OFF (free); force-feedback only when follower detects contact.
//! 6. **Emulated On-Demand** – Leader stays enabled with zero MIT command until contact.
//!
//! All methods run a real-time control loop that communicates with two motors
//! over CAN bus using MIT mode.

use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use std::time::{Duration, Instant};

use socketcan::{CanSocket, Socket};

use crate::driver::MotorSpec;
use crate::error::Result;
use crate::protocol::MotorModel;

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
    /// 5: Leader stays disabled (free); force feedback on follower contact (legacy, use ondemand flag).
    OnDemand,
    /// 6: Leader stays enabled but outputs ~zero torque until contact.
    EmulatedOnDemand,
}

impl BilateralMethod {
    pub const ALL: [BilateralMethod; 6] = [
        BilateralMethod::PositionMirroring,
        BilateralMethod::ForceReflecting,
        BilateralMethod::VirtualCoupling,
        BilateralMethod::ModeSpace,
        BilateralMethod::OnDemand,
        BilateralMethod::EmulatedOnDemand,
    ];

    pub fn label(&self) -> &'static str {
        match self {
            BilateralMethod::PositionMirroring => "Position Mirroring",
            BilateralMethod::ForceReflecting => "Force Reflecting",
            BilateralMethod::VirtualCoupling => "Virtual Coupling",
            BilateralMethod::ModeSpace => "Mode Space (4ch)",
            BilateralMethod::OnDemand => "On-Demand",
            BilateralMethod::EmulatedOnDemand => "Emulated On-Demand",
        }
    }

    pub fn short(&self) -> &'static str {
        match self {
            BilateralMethod::PositionMirroring => "pos",
            BilateralMethod::ForceReflecting => "force",
            BilateralMethod::VirtualCoupling => "coupling",
            BilateralMethod::ModeSpace => "mode",
            BilateralMethod::OnDemand => "ondemand",
            BilateralMethod::EmulatedOnDemand => "ondemand_emu",
        }
    }

    /// Parse from short name.
    pub fn from_short(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "pos" | "position" | "1" => Some(Self::PositionMirroring),
            "force" | "2" => Some(Self::ForceReflecting),
            "coupling" | "virtual" | "3" => Some(Self::VirtualCoupling),
            "mode" | "4ch" | "4" => Some(Self::ModeSpace),
            "ondemand" | "demand" | "od" | "5" => Some(Self::OnDemand),
            "ondemand_emu" | "emulated" | "emu" | "eod" | "6" => {
                Some(Self::EmulatedOnDemand)
            }
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
    /// Maximum velocity-assist torque [Nm].
    /// Limits the motor-internal kd contribution to prevent runaway.
    /// The vel_ref delta is clamped so kd*(vel_ref - vel) <= max_assist.
    /// 0.05 Nm typical (should be less than mechanical friction).
    pub max_assist: f64,
    /// Force threshold for OnDemand mode [Nm].
    /// Leader is enabled only when |follower_torque| > this value.
    /// Hysteresis: disables when < threshold * 0.5.
    pub force_threshold: f64,
    /// Sign of "opening" direction velocity for OnDemand mode.
    /// +1.0 = positive velocity is opening (disable leader).
    /// -1.0 = negative velocity is opening.
    /// 0.0  = direction-based disable is off.
    pub open_sign: f64,
    /// Cutoff frequency [rad/s] for the velocity feedback LPF used by the
    /// coupling kd term. Direct-drive motors (Robstride RS-series, Damiao
    /// DM-series) typically report a velocity that is *itself* internally
    /// filtered with significant phase lag — feeding that lagged signal
    /// into our kd term causes the closed loop to ring at the lag
    /// frequency. A second LPF in software is the wrong cure, but a
    /// numerical-differentiation estimator with a controlled cutoff
    /// gives stable damping. Set <= 0 to bypass (use raw motor velocity).
    /// Typical: 80-200 rad/s.
    pub vel_cutoff: f64,
    /// Maximum allowed change in commanded torque per loop cycle [Nm].
    /// Acts as a slew-rate limiter on the final tau output. When the
    /// virtual-coupling spring slams from "aligned" to "under operator
    /// disturbance" it can ask for a multi-Nm torque step in one CAN
    /// cycle; the motor's current loop overshoots, the operator hand
    /// reflects energy back, and the system rings. Limiting to ~0.5
    /// Nm/cycle (= 250 Nm/s at 500 Hz) softens the leading edge of
    /// transients without slowing steady-state response noticeably.
    /// Set <= 0 to disable.
    pub tau_slew: f64,
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
            max_assist: 0.05,
            force_threshold: 0.3,
            open_sign: 0.0,
            // Off by default: the LPF estimator helps direct-drive motors
            // with internally-lagged velocity (some Robstride models) but
            // *hurts* high-resolution encoders (DAMIAO DM-series) where
            // the motor's own velocity report is far smoother than what
            // numerical position differentiation at 500 Hz can produce.
            // Users opt in per-motor.
            vel_cutoff: 0.0,
            // Off by default: a hard slew limit alters steady-state
            // response and shows up as visible torque steps. Only useful
            // when the operator-grab transient excites a resonance you
            // can't damp another way.
            tau_slew: 0.0,
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
// Driver construction
// =============================================================================
//
// Concrete CAN protocol details live in [`crate::driver`]. The bilateral loops
// only see a `Box<dyn MotorDriver>` per motor, built from the per-motor
// [`MotorSpec`] in the config.

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
///
/// Leader and follower are configured independently via [`MotorSpec`], so any
/// combination of vendors / models can be used.
#[derive(Debug, Clone)]
pub struct BilateralConfig {
    pub interface: String,
    pub leader: MotorSpec,
    pub follower: MotorSpec,
    pub method: BilateralMethod,
    pub ondemand: bool,
    pub gains: BilateralGains,
    /// Target loop period [µs]
    pub loop_period_us: u64,
    /// Safety position bound [rad]. If either motor's soft-zero-adjusted
    /// position exceeds ±this value, the loop aborts and disables both
    /// motors. Set <= 0 to disable. Default π (≈3.14) catches runaway from
    /// assist-loop positive feedback / vendor sign mismatch / etc. before
    /// the joint hits a hard mechanical stop.
    pub safety_radius: f64,
    /// Maximum allowed position change between consecutive samples [rad].
    /// Catches "motor was driven uncontrolled during a CAN dropout" — when
    /// frames are missed under high-rate motion the loop sees prev_pos stuck
    /// at the last good value and the next fresh frame can show the motor
    /// already saturated at ±p_max. Physically a DM-J4310 maxes at ~30 rad/s,
    /// so at a 500 Hz loop the max plausible jump is ~0.06 rad. Default 0.5
    /// rad gives ample slack while still catching multi-rad jumps. Set 0 to
    /// disable.
    pub safety_max_jump: f64,
}

impl Default for BilateralConfig {
    fn default() -> Self {
        BilateralConfig {
            interface: "can0".to_string(),
            leader: MotorSpec::robstride(0xFD, 10, MotorModel::Rs05),
            follower: MotorSpec::robstride(0xFD, 1, MotorModel::Rs05),
            method: BilateralMethod::VirtualCoupling,
            ondemand: false,
            gains: BilateralGains::default(),
            loop_period_us: 2000, // 500 Hz target
            safety_radius: std::f64::consts::PI,
            safety_max_jump: 0.5,
        }
    }
}

/// Compute friction compensation feedforward torque.
///
/// Returns a torque in the direction of motion to overcome internal friction:
///   τ_comp = coulomb·smooth_sign(ω) + viscous·ω
///
/// The Coulomb term uses a **smooth** sign approximation rather than the
/// classical `sign()` step. A pure step at low velocity is the textbook
/// cause of bilateral-coupling limit cycles: the moment the operator nudges
/// the leader, its velocity wanders through ±deadband, the friction comp
/// snaps ±coulomb on/off every other sample, and the closed loop rings at
/// whatever frequency the coupling permits. Linear ramp through ±deadband
/// gives the same DC behaviour without the discontinuity.
fn friction_compensation(velocity: f64, coulomb: f64, viscous: f64) -> f64 {
    // Smooth sign: linear ramp between -DEADBAND and +DEADBAND, then ±1.
    // 0.1 rad/s width is small enough to look like proper friction comp
    // at moderate speeds but wide enough to absorb the encoder velocity
    // jitter that drives the limit cycle.
    const DEADBAND: f64 = 0.1;
    let smooth_sign = (velocity / DEADBAND).clamp(-1.0, 1.0);
    coulomb * smooth_sign + viscous * velocity
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
        let result = match config.method {
            BilateralMethod::OnDemand => run_ondemand_loop(&config, &telem, &stop_flag),
            BilateralMethod::EmulatedOnDemand => {
                run_emulated_ondemand_loop(&config, &telem, &stop_flag)
            }
            _ => run_bilateral_loop(&config, &telem, &stop_flag),
        };
        if let Err(e) = result {
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

    let mut leader = config.leader.build();
    let mut follower = config.follower.build();

    // Enable both motors
    leader.enable(&socket)?;
    std::thread::sleep(Duration::from_millis(20));
    follower.enable(&socket)?;
    std::thread::sleep(Duration::from_millis(20));

    // Initial status read (MIT zero command)
    let fb_l = leader.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0)?;
    let fb_f = follower.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0)?;

    // Pre-flight position sanity check. If either motor is already well
    // outside the safety radius at launch, refuse to engage and tell the
    // operator how to fix it. Without this the in-loop radius watchdog
    // trips on cycle 0 — before any telemetry is published — and the user
    // just sees a fully-zero TUI with a single ERR line, which is hard to
    // interpret. `2.0 *` margin so we don't refuse for tiny excursions
    // that the soft-start can quickly pull back; truly far-away motors
    // (e.g. multi-turn pose with no Zero Pair run) get rejected here.
    if config.safety_radius > 0.0 {
        let limit = config.safety_radius * 2.0;
        if fb_l.position.abs() > limit || fb_f.position.abs() > limit {
            let _ = leader.disable(&socket);
            let _ = follower.disable(&socket);
            if let Ok(mut t) = telemetry.lock() {
                t.last_error = Some(format!(
                    "STARTUP: motor outside 2×safety_rad before loop \
                     (leader pos={:.3}, follower pos={:.3}, safety_rad={:.2}). \
                     Run 'Zero Pair' or move the motor closer to zero, or raise safety_rad.",
                    fb_l.position, fb_f.position, config.safety_radius,
                ));
            }
            return Ok(());
        }
    }

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

    // Software velocity estimators. We differentiate position with an LPF
    // because the raw motor-reported velocity is internally smoothed by an
    // unknown filter and carries phase lag we cannot afford in the kd
    // path. If `vel_cutoff <= 0` we fall back to the motor's reported
    // velocity (legacy behaviour).
    let use_vel_estimator = config.gains.vel_cutoff > 0.0;
    let vel_cutoff = if use_vel_estimator { config.gains.vel_cutoff } else { 0.0 };
    let mut vel_lpf_l = LowPassFilter::new(vel_cutoff);
    let mut vel_lpf_f = LowPassFilter::new(vel_cutoff);
    // Seed the filter outputs at the current measured velocity so the
    // first few cycles don't see a step from 0 to the actual speed.
    vel_lpf_l.output = fb_l.velocity;
    vel_lpf_f.output = fb_f.velocity;
    let mut prev_l_vel_filt: f64 = fb_l.velocity;
    let mut prev_f_vel_filt: f64 = fb_f.velocity;

    // Slew-rate limited previous output torques (for rate limiter).
    let mut prev_tau_l_cmd: f64 = 0.0;
    let mut prev_tau_f_cmd: f64 = 0.0;
    let tau_slew = config.gains.tau_slew.max(0.0);

    // Per-motor torque clamps (50 % of model scale is already applied by the
    // Robstride driver; other vendors apply their own margin).
    let leader_torque_limit = leader.torque_limit();
    let follower_torque_limit = follower.torque_limit();
    let start_time = Instant::now();

    // Per-motor "last successful exchange" timestamps. The jump watchdog
    // uses the elapsed wall time since the last good frame so that a CAN
    // dropout (which leaves prev_* stale) doesn't false-trip on the next
    // good read: at high speed the legitimate position delta across a
    // 10 ms timeout window can already approach the static threshold.
    let mut last_good_l = Instant::now();
    let mut last_good_f = Instant::now();
    // Plausible maximum joint velocity used to scale the jump threshold
    // with elapsed time. Bumped to 150 rad/s: the MIT velocity scales we
    // ship max out at 50 rad/s but those are the *commanded* limits — a
    // human back-driving the leader (or a low-friction direct-drive motor
    // whose rotor is geared up to the joint) can easily exceed that for
    // brief flicks, and we don't want every wrist-flick to trip safety.
    const SAFETY_MAX_PLAUSIBLE_VEL: f64 = 150.0;

    // Track previous iteration start so loop_hz reports actual loop rate
    // (work + sleep) rather than the misleading "work-only" rate.
    let mut prev_iter_start: Option<Instant> = None;

    // Spurious-frame counter (see in-loop docstring on the rejection
    // logic). Trips safety only after MAX consecutive bad frames; a single
    // clean frame resets the counter to 0.
    let mut spurious_count: u32 = 0;
    const MAX_SPURIOUS_BEFORE_TRIP: u32 = 20;

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

        // Choose velocity signal for the kd path: either the LPF-smoothed
        // numerical derivative of position, or the motor's raw velocity
        // (legacy). The estimator avoids the unknown internal LPF lag in
        // the motor-reported velocity, which is the dominant destabiliser
        // for bilateral coupling on direct-drive motors.
        let (vel_l_eff, vel_f_eff) = if use_vel_estimator {
            (prev_l_vel_filt, prev_f_vel_filt)
        } else {
            (prev_l_vel, prev_f_vel)
        };

        // Compute torques based on method
        let (tau_leader, tau_follower) = match config.method {
            BilateralMethod::PositionMirroring => {
                let tau_l = -0.05 * vel_l_eff;
                let err = prev_l_pos - prev_f_pos;
                let derr = vel_l_eff - vel_f_eff;
                let tau_f = kp * err + kd * derr;
                (tau_l, tau_f)
            }
            BilateralMethod::ForceReflecting => {
                let err = prev_l_pos - prev_f_pos;
                let derr = vel_l_eff - vel_f_eff;
                let tau_f = kp * err + kd * derr;
                let tau_l = -config.gains.force_scale * follower_torque_est - 0.05 * vel_l_eff;
                (tau_l, tau_f)
            }
            BilateralMethod::VirtualCoupling => {
                let err = prev_l_pos - prev_f_pos;
                let derr = vel_l_eff - vel_f_eff;
                let coupling = kp * err + kd * derr;
                (-coupling, coupling)
            }
            BilateralMethod::ModeSpace => {
                let tau_ext_l = dob_leader.update(0.0, vel_l_eff, dt);
                let tau_ext_f = dob_follower.update(0.0, vel_f_eff, dt);
                let pos_err = prev_l_pos - prev_f_pos;
                let vel_err = vel_l_eff - vel_f_eff;
                let tau_diff = kp * pos_err + kd * vel_err;
                let tau_l = -tau_diff + tau_ext_f;
                let tau_f = tau_diff + tau_ext_l;
                (tau_l, tau_f)
            }
            BilateralMethod::OnDemand => {
                unreachable!("OnDemand uses run_ondemand_loop")
            }
            BilateralMethod::EmulatedOnDemand => {
                unreachable!("EmulatedOnDemand uses run_emulated_ondemand_loop")
            }
        };
        // OnDemand gating (all methods except OnDemand legacy)
        let mut leader_enabled = true;
        if config.ondemand && config.method != BilateralMethod::OnDemand {
            let follower_force = follower_torque_est.abs();
            let threshold = config.gains.force_threshold.abs().max(0.01);
            leader_enabled = follower_force > threshold;
        }

        // Friction compensation feedforward for each motor.
        // This cancels internal motor friction so it does not propagate
        // through the virtual coupling as a phantom force.
        //   τ_comp = coulomb·smooth_sign(ω) + viscous·ω
        // Applied in the direction of existing velocity to overcome friction.
        // Use the smoothed velocity to avoid re-introducing the
        // quantisation/jitter we filter out for kd.
        let friction_comp_l = friction_compensation(vel_l_eff, coulomb, viscous);
        let friction_comp_f = friction_compensation(vel_f_eff, coulomb, viscous);

        // Leader inertia compensation: τ = -J_comp · α_filtered
        // This makes the leader feel lighter by cancelling its own inertia.
        let raw_accel = if dt > 0.0 {
            (vel_l_eff - leader_prev_vel) / dt
        } else {
            0.0
        };
        leader_prev_vel = vel_l_eff;
        let filtered_accel = accel_lpf.update(raw_accel, dt);
        let inertia_comp_torque = -j_comp * filtered_accel;

        // Apply soft-start ramp to *all* torques. The primary coupling
        // torque must be ramped too: if the leader/follower aren't
        // perfectly aligned at launch, an un-ramped kp·err would inject a
        // step torque at t=0 and ring the closed loop (the symptom users
        // see as "coupling becomes oscillatory right after starting").
        let tau_leader_total =
            tau_leader * ramp + (friction_comp_l + inertia_comp_torque) * ramp;
        let tau_follower_total = tau_follower * ramp + friction_comp_f * ramp;

        // Slew-rate limit the commanded torque. Bilateral coupling tends
        // to produce step torques when the operator suddenly grabs either
        // side: kp · err jumps by several Nm in one cycle, the motor's
        // current loop overshoots, mechanical reflection rings. Capping
        // the per-cycle change to `tau_slew` Nm spreads that step over a
        // few ms and removes the leading edge that excites the lightly-
        // damped modes of the joint + operator-hand system. `tau_slew=0`
        // disables (legacy behaviour).
        let (tau_leader_slewed, tau_follower_slewed) = if tau_slew > 0.0 {
            let dl = (tau_leader_total - prev_tau_l_cmd).clamp(-tau_slew, tau_slew);
            let df = (tau_follower_total - prev_tau_f_cmd).clamp(-tau_slew, tau_slew);
            (prev_tau_l_cmd + dl, prev_tau_f_cmd + df)
        } else {
            (tau_leader_total, tau_follower_total)
        };

        // Clamp
        let tau_leader_clamped =
            tau_leader_slewed.clamp(-leader_torque_limit, leader_torque_limit);
        let tau_follower_clamped =
            tau_follower_slewed.clamp(-follower_torque_limit, follower_torque_limit);
        prev_tau_l_cmd = tau_leader_clamped;
        prev_tau_f_cmd = tau_follower_clamped;

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
            // Clamp vel delta so assist torque <= max_assist
            let kd_ramped = config.gains.assist_kd * ramp;
            let vel_delta = prev_l_vel * (config.gains.vel_ahead - 1.0);
            let max_delta = if kd_ramped > 0.0 {
                config.gains.max_assist / kd_ramped
            } else {
                0.0
            };
            let vel_ref = prev_l_vel + vel_delta.clamp(-max_delta, max_delta);
            (kd_ramped, vel_ref)
        } else {
            (0.0, 0.0)
        };
        let leader_vel_assist_est = if config.gains.assist_kd > 0.0 {
            let vel_delta = prev_l_vel * (config.gains.vel_ahead - 1.0);
            let kd_ramped = config.gains.assist_kd * ramp;
            let max_delta = if kd_ramped > 0.0 {
                config.gains.max_assist / kd_ramped
            } else {
                0.0
            };
            kd_ramped * vel_delta.clamp(-max_delta, max_delta)
        } else {
            0.0
        };

        // Send MIT commands
        let fb_l = match leader.mit_exchange(
            &socket, 0.0, mit_vel_leader, 0.0, mit_kd_leader, tau_leader_clamped,
        ) {
            Ok(fb) => fb,
            Err(_e) => {
                // On timeout, keep previous values
                cycle += 1;
                continue;
            }
        };

        let fb_f = match follower.mit_exchange(
            &socket, 0.0, 0.0, 0.0, 0.0, tau_follower_clamped,
        ) {
            Ok(fb) => fb,
            Err(_e) => {
                cycle += 1;
                continue;
            }
        };

        // Spurious-frame filter.
        //
        // Occasionally a single CAN status frame comes back with a
        // wildly-wrong position even though the motor is essentially
        // stationary (velocity ~ 0). The root cause is unclear (kernel
        // buffer reordering across enable / parameter reads, a stale
        // active-report frame from before set_soft_zero ran, transceiver
        // glitch, ...) but the signature is clear: |Δpos| is far larger
        // than what fb.velocity·Δt could physically explain.
        //
        // If we hand such a frame to the safety_radius check it trips
        // immediately on a value that doesn't represent reality. Instead,
        // treat the frame itself as suspect, keep prev_* unchanged, and
        // try again next cycle. After too many consecutive rejects we
        // *do* trip, because at that point the comms are genuinely broken.
        let now = Instant::now();
        let dt_l_phys = now.duration_since(last_good_l).as_secs_f64();
        let dt_f_phys = now.duration_since(last_good_f).as_secs_f64();
        // Bound the plausible per-frame delta by the *average* of the
        // two velocity samples (current and previous) times elapsed time,
        // plus a small slack to absorb encoder quantisation noise.
        const SPURIOUS_SLACK_RAD: f64 = 0.1;
        let plausible_dl = 0.5 * (fb_l.velocity.abs() + prev_l_vel.abs()) * dt_l_phys
            + SPURIOUS_SLACK_RAD;
        let plausible_df = 0.5 * (fb_f.velocity.abs() + prev_f_vel.abs()) * dt_f_phys
            + SPURIOUS_SLACK_RAD;
        let dl_now = (fb_l.position - prev_l_pos).abs();
        let df_now = (fb_f.position - prev_f_pos).abs();
        let spurious_l = cycle > 0 && dl_now > plausible_dl * 5.0 + 0.3;
        let spurious_f = cycle > 0 && df_now > plausible_df * 5.0 + 0.3;
        if spurious_l || spurious_f {
            spurious_count += 1;
            // Surface a soft warning so users can see the frame went
            // through the rejection path (cleared by the next clean cycle).
            if let Ok(mut t) = telemetry.lock() {
                t.last_error = Some(format!(
                    "WARN spurious frame #{} (leader pos={:.3} v={:.3} Δ={:.3}>{:.3}; \
                     follower pos={:.3} v={:.3} Δ={:.3}>{:.3}) — dropped",
                    spurious_count,
                    fb_l.position, fb_l.velocity, dl_now, plausible_dl * 5.0 + 0.3,
                    fb_f.position, fb_f.velocity, df_now, plausible_df * 5.0 + 0.3,
                ));
            }
            if spurious_count >= MAX_SPURIOUS_BEFORE_TRIP {
                if let Ok(mut t) = telemetry.lock() {
                    t.last_error = Some(format!(
                        "SAFETY: {} consecutive spurious frames — disabling \
                         (last leader pos={:.3} v={:.3}, follower pos={:.3} v={:.3})",
                        spurious_count,
                        fb_l.position, fb_l.velocity,
                        fb_f.position, fb_f.velocity,
                    ));
                }
                break;
            }
            // Don't update prev_* / last_good_* and don't run the safety
            // checks on a frame we just rejected. Keep cycling.
            cycle += 1;
            let work_time = iter_start.elapsed();
            if work_time < loop_period {
                std::thread::sleep(loop_period - work_time);
            }
            continue;
        }
        spurious_count = 0;

        // Jump watchdog: catch the case where CAN dropouts kept prev_*
        // frozen while the motor was physically accelerating, so by the time
        // a fresh frame arrives the position has already moved into a
        // dangerous regime. Skip on cycle 0 — prev_* was seeded from the
        // pre-loop initial exchange and the very first iteration's delta is
        // meaningless if the read timed out then.
        //
        // The threshold is wall-clock aware: it scales with how long it has
        // been since the last successful read of *that* motor, with
        // `safety_max_jump` as a floor. Without this, a single CAN timeout
        // (~10 ms) under high-speed motion makes |Δpos| approach the static
        // limit (33 rad/s × 10 ms ≈ 0.33 rad on RS-05) and produces a
        // false-positive trip the instant the next good frame arrives —
        // which is the "starts and immediately stops with SAFETY error"
        // symptom in coupling mode.
        //
        // During the soft-start ramp (first SOFT_START_SECS) the loop
        // commands only a fraction of the configured torque, so the motor
        // cannot be driven into a runaway by *us*; meanwhile the operator
        // is typically grabbing/aligning the leader by hand and producing
        // high transient velocities. Skip the jump check during that
        // window to avoid tripping on legitimate manual motion.
        let in_softstart = ramp < 1.0;
        if cycle > 0 && config.safety_max_jump > 0.0 && !in_softstart {
            let dt_l = now.duration_since(last_good_l).as_secs_f64();
            let dt_f = now.duration_since(last_good_f).as_secs_f64();
            let thr_l = config
                .safety_max_jump
                .max(SAFETY_MAX_PLAUSIBLE_VEL * dt_l);
            let thr_f = config
                .safety_max_jump
                .max(SAFETY_MAX_PLAUSIBLE_VEL * dt_f);
            let dl = (fb_l.position - prev_l_pos).abs();
            let df = (fb_f.position - prev_f_pos).abs();
            if dl > thr_l || df > thr_f {
                if let Ok(mut t) = telemetry.lock() {
                    t.last_error = Some(format!(
                        "SAFETY: position jump exceeds plausible motion \
                         (leader Δ={:.3}>{:.3} or follower Δ={:.3}>{:.3}) — disabling",
                        dl, thr_l, df, thr_f,
                    ));
                }
                break;
            }
        }
        last_good_l = now;
        last_good_f = now;

        // Update the LPF velocity estimators from the position derivative.
        // Use the *new* fb position vs the *previous* prev_pos and the
        // measured dt. The LPF rejects per-cycle quantisation jitter while
        // keeping the dominant motion content. The filter output is what
        // the next iteration's kd term will see (`vel_l_eff` / `vel_f_eff`).
        if use_vel_estimator && dt > 0.0 {
            let dl = (fb_l.position - prev_l_pos) / dt;
            let df = (fb_f.position - prev_f_pos) / dt;
            prev_l_vel_filt = vel_lpf_l.update(dl, dt);
            prev_f_vel_filt = vel_lpf_f.update(df, dt);
        } else {
            prev_l_vel_filt = fb_l.velocity;
            prev_f_vel_filt = fb_f.velocity;
        }

        prev_l_pos = fb_l.position;
        prev_l_vel = fb_l.velocity;
        prev_f_pos = fb_f.position;
        prev_f_vel = fb_f.velocity;

        // Safety watchdog: bail out before either motor walks into a hard
        // mechanical stop. Runaway typically comes from assist-loop positive
        // feedback (low-friction DM leader) or a sign-convention mismatch
        // between leader and follower.
        //
        // Skip during soft-start: at launch the operator may have left a
        // motor parked outside ±safety_radius (multi-turn position, no
        // recent Zero Pair, etc.). The ramped torque during soft-start
        // pulls the two motors toward each other so the leader naturally
        // moves toward the safety zone within the ramp window. Only after
        // ramp = 1.0 does an out-of-zone reading represent a genuine
        // failure to track / runaway we should react to.
        if config.safety_radius > 0.0
            && !in_softstart
            && (fb_l.position.abs() > config.safety_radius
                || fb_f.position.abs() > config.safety_radius)
        {
            if let Ok(mut t) = telemetry.lock() {
                t.last_error = Some(format!(
                    "SAFETY: |pos| exceeded {:.2} rad after soft-start \
                     (leader pos={:.3} v={:.3} τ={:.3}, follower pos={:.3} v={:.3} τ={:.3}) — \
                     disabling. Tip: run 'Zero Pair' first, increase safety_rad, \
                     or move motor closer to zero before starting.",
                    config.safety_radius,
                    fb_l.position, fb_l.velocity, fb_l.torque,
                    fb_f.position, fb_f.velocity, fb_f.torque,
                ));
            }
            break;
        }

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

        // Compute loop frequency (actual loop period including sleep, not
        // just work time). Skip the first cycle since prev_iter_start is
        // None and the very first interval isn't representative anyway.
        if let Some(prev) = prev_iter_start {
            let cycle_secs = iter_start.duration_since(prev).as_secs_f64();
            if cycle_secs > 0.0 {
                hz_accum += 1.0 / cycle_secs;
                hz_count += 1;
            }
        }
        prev_iter_start = Some(iter_start);

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
    let _ = leader.disable(&socket);
    let _ = follower.disable(&socket);

    Ok(())
}

// =============================================================================
// On-Demand Force Feedback
// =============================================================================
//
// Leader motor stays DISABLED (completely free backdrive).
// Position is read via parameter read (works when disabled).
// Follower tracks leader position with PD control (MIT mode).
// When follower detects reaction force (|torque| > threshold),
// leader is ENABLED and receives reflected force.
// When force drops below threshold*0.5 (hysteresis), leader is disabled again.

fn run_ondemand_loop(
    config: &BilateralConfig,
    telemetry: &SharedTelemetry,
    stop: &StopFlag,
) -> Result<()> {
    let socket = CanSocket::open(&config.interface)?;
    socket.set_read_timeout(Duration::from_millis(10))?;

    let mut leader = config.leader.build();
    let mut follower = config.follower.build();

    // Leader starts DISABLED (free to backdrive)
    let _ = leader.disable(&socket);
    std::thread::sleep(Duration::from_millis(20));

    // Follower enabled in MIT mode
    follower.enable(&socket)?;
    std::thread::sleep(Duration::from_millis(20));

    // Read initial leader position via param read
    let l_pos_init = leader.read_position(&socket)?;
    let l_vel_init = leader.read_velocity(&socket)?;

    // Follower initial status (MIT zero)
    let fb_f = follower.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0)?;

    let mut prev_l_pos = l_pos_init;
    let mut prev_l_vel = l_vel_init;
    let mut _prev_f_pos = fb_f.position;
    let mut prev_f_vel = fb_f.velocity;

    let kp = config.gains.kp;
    let kd = config.gains.kd;
    let force_threshold = config.gains.force_threshold.abs().max(0.01);
    let force_scale = config.gains.force_scale;
    let open_sign = config.gains.open_sign;
    let coulomb = config.gains.coulomb_friction;
    let viscous = config.gains.viscous_friction;
    let loop_period = Duration::from_micros(config.loop_period_us);

    let leader_torque_limit = leader.torque_limit();
    let follower_torque_limit = follower.torque_limit();

    let mut leader_enabled = false;
    let mut cycle: u64 = 0;
    let mut loop_start = Instant::now();
    let mut hz_accum = 0.0;
    let mut hz_count = 0u32;
    let start_time = Instant::now();

    loop {
        if stop.load(Ordering::Relaxed) {
            break;
        }

        let iter_start = Instant::now();
        let ramp = soft_start_gain(start_time.elapsed().as_secs_f64());
        let _dt = if cycle == 0 {
            loop_period.as_secs_f64()
        } else {
            iter_start.duration_since(loop_start).as_secs_f64().max(0.0001)
        };
        loop_start = iter_start;

        // --- Read leader position (works when disabled) ---
        let l_pos = leader.read_position(&socket).unwrap_or(prev_l_pos);
        let l_vel = leader.read_velocity(&socket).unwrap_or(prev_l_vel);
        prev_l_pos = l_pos;
        prev_l_vel = l_vel;

        // --- Follower: MIT internal position tracking of leader (10kHz) ---
        // Motor internally computes: τ = kp*(pos_ref - pos) + kd*(vel_ref - vel) + τ_ff
        // This runs at motor's internal rate (~10kHz), much stiffer than CAN-rate torque.
        let mit_kp_f = kp * ramp;
        let mit_kd_f = kd * ramp;
        let friction_comp_f = friction_compensation(prev_f_vel, coulomb, viscous) * ramp;
        let tau_ff_f = friction_comp_f.clamp(-follower_torque_limit, follower_torque_limit);

        let fb_f = match follower.mit_exchange(
            &socket, l_pos, l_vel, mit_kp_f, mit_kd_f, tau_ff_f,
        ) {
            Ok(fb) => fb,
            Err(_) => {
                cycle += 1;
                continue;
            }
        };

        // Estimate the total torque the follower is actually applying
        // (used for force detection and telemetry)
        let tau_follower_est = mit_kp_f * (l_pos - fb_f.position)
            + mit_kd_f * (l_vel - fb_f.velocity)
            + tau_ff_f;

        // Jump watchdog (see run_bilateral_loop for rationale).
        if cycle > 0 && config.safety_max_jump > 0.0 {
            let dl = (l_pos - prev_l_pos).abs();
            let df = (fb_f.position - _prev_f_pos).abs();
            if dl > config.safety_max_jump || df > config.safety_max_jump {
                if let Ok(mut t) = telemetry.lock() {
                    t.last_error = Some(format!(
                        "SAFETY: position jumped > {:.2} rad/cycle (leader Δ={:.3}, follower Δ={:.3}) — \
                         likely CAN dropout under motion; disabling",
                        config.safety_max_jump, dl, df,
                    ));
                }
                break;
            }
        }

        _prev_f_pos = fb_f.position;
        prev_f_vel = fb_f.velocity;

        // Safety watchdog (see run_bilateral_loop for rationale).
        if config.safety_radius > 0.0
            && (l_pos.abs() > config.safety_radius
                || fb_f.position.abs() > config.safety_radius)
        {
            if let Ok(mut t) = telemetry.lock() {
                t.last_error = Some(format!(
                    "SAFETY: |pos| exceeded {:.2} rad (leader={:.3}, follower={:.3}) — disabling",
                    config.safety_radius, l_pos, fb_f.position,
                ));
            }
            break;
        }

        // --- Detect follower reaction force ---
        // Position error indicates blocked motion; torque feedback confirms contact
        let follower_force = fb_f.torque.abs();

        // --- Leader enable/disable logic with hysteresis ---
        let mut tau_leader_cmd: f64 = 0.0;

        // Direction-based enable gate: if leader moves in "opening" direction,
        // prevent enable (keeps leader free). Does NOT force-disable during
        // active contact to avoid ON/OFF oscillation from reaction torque.
        let opening = open_sign != 0.0 && (l_vel * open_sign) > 0.1;

        if !leader_enabled && !opening && follower_force > force_threshold {
            // Contact detected -> enable leader for force feedback
            leader.enable(&socket)?;
            std::thread::sleep(Duration::from_millis(5));
            leader_enabled = true;
        }

        if leader_enabled {
            if follower_force < force_threshold * 0.5 {
                // Contact released -> disable leader (free again)
                let _ = leader.disable(&socket);
                leader_enabled = false;
            } else {
                // --- Leader: torque reflection from follower ---
                // Reflects the actual environment force measured by follower.
                // This preserves the true impedance of the contacted object:
                //   soft object → small follower torque → soft feel at leader
                //   hard object → large follower torque → hard feel at leader
                //
                // The follower is tracking leader at 10kHz (MIT internal kp/kd),
                // so fb_f.torque accurately represents the environment reaction.
                tau_leader_cmd = (-force_scale * fb_f.torque * ramp)
                    .clamp(-leader_torque_limit, leader_torque_limit);
                let fb_l = match leader.mit_exchange(
                    &socket, 0.0, 0.0, 0.0, 0.0, tau_leader_cmd,
                ) {
                    Ok(fb) => fb,
                    Err(_) => {
                        cycle += 1;
                        continue;
                    }
                };
                // Update leader state from actual feedback
                prev_l_pos = fb_l.position;
                prev_l_vel = fb_l.velocity;
            }
        }

        // Compute loop frequency
        let elapsed = iter_start.elapsed().as_secs_f64();
        if elapsed > 0.0 {
            hz_accum += 1.0 / elapsed;
            hz_count += 1;
        }

        // Update telemetry
        if cycle % 10 == 0 {
            if let Ok(mut t) = telemetry.lock() {
                t.leader_pos = l_pos;
                t.leader_vel = l_vel;
                t.leader_torque_cmd = tau_leader_cmd;
                t.follower_pos = fb_f.position;
                t.follower_vel = fb_f.velocity;
                t.follower_torque_cmd = tau_follower_est;
                t.position_error = prev_l_pos - fb_f.position;
                t.leader_friction_comp = 0.0;
                t.follower_friction_comp = friction_comp_f;
                t.leader_inertia_comp = if leader_enabled { 1.0 } else { 0.0 }; // reuse field as ON/OFF indicator
                t.leader_vel_assist = follower_force; // reuse field to show detected force
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

    // Disable both motors on exit
    let _ = leader.disable(&socket);
    let _ = follower.disable(&socket);

    Ok(())
}

fn run_emulated_ondemand_loop(
    config: &BilateralConfig,
    telemetry: &SharedTelemetry,
    stop: &StopFlag,
) -> Result<()> {
    let socket = CanSocket::open(&config.interface)?;
    socket.set_read_timeout(Duration::from_millis(10))?;

    let mut leader = config.leader.build();
    let mut follower = config.follower.build();

    // Emulated free-leader mode: keep leader enabled and command zero MIT
    // until contact is detected.
    leader.enable(&socket)?;
    std::thread::sleep(Duration::from_millis(20));
    follower.enable(&socket)?;
    std::thread::sleep(Duration::from_millis(20));

    let fb_l = leader.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0)?;
    let fb_f = follower.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0)?;

    let mut prev_l_pos = fb_l.position;
    let mut prev_l_vel = fb_l.velocity;
    let mut prev_f_vel = fb_f.velocity;

    let kp = config.gains.kp;
    let kd = config.gains.kd;
    let force_threshold = config.gains.force_threshold.abs().max(0.01);
    let force_scale = config.gains.force_scale;
    let open_sign = config.gains.open_sign;
    let coulomb = config.gains.coulomb_friction;
    let viscous = config.gains.viscous_friction;
    let loop_period = Duration::from_micros(config.loop_period_us);

    let leader_torque_limit = leader.torque_limit();
    let follower_torque_limit = follower.torque_limit();

    let mut reflect_enabled = false;
    let mut cycle: u64 = 0;
    let mut loop_start = Instant::now();
    let mut hz_accum = 0.0;
    let mut hz_count = 0u32;
    let start_time = Instant::now();

    loop {
        if stop.load(Ordering::Relaxed) {
            break;
        }

        let iter_start = Instant::now();
        let ramp = soft_start_gain(start_time.elapsed().as_secs_f64());
        let _dt = if cycle == 0 {
            loop_period.as_secs_f64()
        } else {
            iter_start.duration_since(loop_start).as_secs_f64().max(0.0001)
        };
        loop_start = iter_start;

        // Follower tracks current leader state.
        let mit_kp_f = kp * ramp;
        let mit_kd_f = kd * ramp;
        let friction_comp_f = friction_compensation(prev_f_vel, coulomb, viscous) * ramp;
        let tau_ff_f = friction_comp_f.clamp(-follower_torque_limit, follower_torque_limit);

        let fb_f = match follower.mit_exchange(
            &socket,
            prev_l_pos,
            prev_l_vel,
            mit_kp_f,
            mit_kd_f,
            tau_ff_f,
        ) {
            Ok(fb) => fb,
            Err(_) => {
                cycle += 1;
                continue;
            }
        };

        let tau_follower_est = mit_kp_f * (prev_l_pos - fb_f.position)
            + mit_kd_f * (prev_l_vel - fb_f.velocity)
            + tau_ff_f;
        let follower_force = fb_f.torque.abs();

        let opening = open_sign != 0.0 && (prev_l_vel * open_sign) > 0.1;
        if !reflect_enabled && !opening && follower_force > force_threshold {
            reflect_enabled = true;
        }
        if reflect_enabled && follower_force < force_threshold * 0.5 {
            reflect_enabled = false;
        }

        let tau_leader_cmd = if reflect_enabled {
            (-force_scale * fb_f.torque * ramp).clamp(-leader_torque_limit, leader_torque_limit)
        } else {
            0.0
        };

        let fb_l = match leader.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, tau_leader_cmd) {
            Ok(fb) => fb,
            Err(_) => {
                cycle += 1;
                continue;
            }
        };

        prev_l_pos = fb_l.position;
        prev_l_vel = fb_l.velocity;
        prev_f_vel = fb_f.velocity;

        if config.safety_radius > 0.0
            && (prev_l_pos.abs() > config.safety_radius
                || fb_f.position.abs() > config.safety_radius)
        {
            if let Ok(mut t) = telemetry.lock() {
                t.last_error = Some(format!(
                    "SAFETY: |pos| exceeded {:.2} rad (leader={:.3}, follower={:.3}) — disabling",
                    config.safety_radius, prev_l_pos, fb_f.position,
                ));
            }
            break;
        }

        let elapsed = iter_start.elapsed().as_secs_f64();
        if elapsed > 0.0 {
            hz_accum += 1.0 / elapsed;
            hz_count += 1;
        }

        if cycle % 10 == 0 {
            if let Ok(mut t) = telemetry.lock() {
                t.leader_pos = prev_l_pos;
                t.leader_vel = prev_l_vel;
                t.leader_torque_cmd = tau_leader_cmd;
                t.follower_pos = fb_f.position;
                t.follower_vel = fb_f.velocity;
                t.follower_torque_cmd = tau_follower_est;
                t.position_error = prev_l_pos - fb_f.position;
                t.leader_friction_comp = 0.0;
                t.follower_friction_comp = friction_comp_f;
                t.leader_inertia_comp = if reflect_enabled { 1.0 } else { 0.0 };
                t.leader_vel_assist = follower_force;
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

    let _ = leader.disable(&socket);
    let _ = follower.disable(&socket);

    Ok(())
}

// =============================================================================
// Single-motor Assist Test
// =============================================================================

/// Configuration for the single-motor assist test.
#[derive(Debug, Clone)]
pub struct AssistTestConfig {
    pub interface: String,
    pub motor: MotorSpec,
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
    /// Maximum velocity-assist torque [Nm].
    /// Limits the motor-internal kd contribution to prevent runaway.
    pub max_assist: f64,
    /// Target loop period [µs]
    pub loop_period_us: u64,
}

impl Default for AssistTestConfig {
    fn default() -> Self {
        AssistTestConfig {
            interface: "can0".to_string(),
            motor: MotorSpec::robstride(0xFD, 10, MotorModel::Rs05),
            assist_kd: 0.0,
            vel_ahead: 2.0,
            coulomb_friction: 0.0,
            viscous_friction: 0.0,
            inertia: 0.005,
            inertia_comp: 0.0,
            accel_cutoff: 50.0,
            max_assist: 0.05,
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

    let mut motor = config.motor.build();

    // Enable motor
    motor.enable(&socket)?;
    std::thread::sleep(Duration::from_millis(20));

    // Initial status read
    let fb = motor.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0)?;
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

    let torque_limit = motor.torque_limit();
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
            let kd_ramped = config.assist_kd * ramp;
            let vel_delta = prev_vel * (config.vel_ahead - 1.0);
            let max_delta = if kd_ramped > 0.0 {
                config.max_assist / kd_ramped
            } else {
                0.0
            };
            let vel_ref = prev_vel + vel_delta.clamp(-max_delta, max_delta);
            (kd_ramped, vel_ref)
        } else {
            (0.0, 0.0)
        };
        let vel_assist_est = if config.assist_kd > 0.0 {
            let vel_delta = prev_vel * (config.vel_ahead - 1.0);
            let kd_ramped = config.assist_kd * ramp;
            let max_delta = if kd_ramped > 0.0 {
                config.max_assist / kd_ramped
            } else {
                0.0
            };
            kd_ramped * vel_delta.clamp(-max_delta, max_delta)
        } else {
            0.0
        };

        // Send MIT command
        let fb = match motor.mit_exchange(
            &socket, 0.0, mit_vel, 0.0, mit_kd, tau_ff,
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

    let _ = motor.disable(&socket);
    Ok(())
}
