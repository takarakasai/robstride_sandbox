//! Robstride Motor Control TUI
//!
//! Interactive terminal application for controlling Robstride motors via CAN bus.
//! Uses Ratatui + Crossterm for the terminal interface.

use std::collections::HashMap;
use std::io;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::Result;
use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyModifiers};
use crossterm::terminal::{self, EnterAlternateScreen, LeaveAlternateScreen};
use crossterm::ExecutableCommand;
use ratatui::prelude::*;
use ratatui::widgets::*;
use serde::{Deserialize, Serialize};

use robstride_sandbox::bilateral::{self, AssistTestConfig, BilateralConfig, BilateralGains, BilateralMethod, SharedTelemetry, StopFlag};
use robstride_sandbox::driver::{self, DamiaoModel, MotorSpec};
use socketcan::{CanSocket, Socket};
use robstride_sandbox::motor::Motor;
use robstride_sandbox::protocol::{MotorFeedback, MotorModel, ParamIndex, RunMode};

const DM_MOVE_KP: f64 = 20.0;
const DM_MOVE_KD: f64 = 1.0;
const DM_SPIN_KD: f64 = 1.0;

/// Choice string presented to the user for motor kind selection. Keep in
/// sync with [`parse_motor_kind`].
const MOTOR_KIND_CHOICES: &str = "rs00|rs01|rs02|rs03|rs04|rs05|rs06|dm4310";

/// Parse a motor-kind string into a [`MotorSpec`].
///
/// Returns `(spec, warning)`. `warning` is `Some(msg)` if `kind` was not
/// recognised; the caller should surface this to the user instead of silently
/// using the fallback Robstride spec.
fn parse_motor_kind(
    kind: &str,
    id_str: &str,
    default_id: u8,
    host_id: u8,
    default_model: MotorModel,
) -> (MotorSpec, Option<String>) {
    let id: u8 = id_str.parse().unwrap_or(default_id);
    if let Some(model) = MotorModel::from_str(kind) {
        return (MotorSpec::robstride(host_id, id, model), None);
    }
    if let Some(model) = DamiaoModel::from_str_ci(kind) {
        // master_id = 0 means "match any standard ID, filter by payload nibble".
        return (MotorSpec::damiao(id, 0, model), None);
    }
    let warn = format!(
        "Unknown motor kind '{}', falling back to Robstride {} (choices: {})",
        kind, default_model, MOTOR_KIND_CHOICES
    );
    (MotorSpec::robstride(host_id, id, default_model), Some(warn))
}

const APP_NAME: &str = "robstride_sandbox";

// =============================================================================
// Persistent config
// =============================================================================

/// Saved parameter values, keyed by command label then param name.
/// Stored in `~/.config/robstride_sandbox/default-config.toml`.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct AppConfig {
    /// { "Bilateral" => { "kp" => "5.0", "kd" => "0.3", ... }, ... }
    params: HashMap<String, HashMap<String, String>>,
}

impl Default for AppConfig {
    fn default() -> Self {
        AppConfig {
            params: HashMap::new(),
        }
    }
}

impl AppConfig {
    fn load() -> Self {
        confy::load(APP_NAME, None).unwrap_or_default()
    }

    fn save(&self) {
        if let Err(e) = confy::store(APP_NAME, None, self) {
            eprintln!("Config save error: {}", e);
        }
    }

    /// Get saved value for a command param, if it exists.
    fn get_param(&self, cmd_label: &str, param_name: &str) -> Option<&String> {
        self.params.get(cmd_label)?.get(param_name)
    }

    /// Set a saved value for a command param.
    fn set_param(&mut self, cmd_label: &str, param_name: &str, value: &str) {
        self.params
            .entry(cmd_label.to_string())
            .or_default()
            .insert(param_name.to_string(), value.to_string());
    }
}

// =============================================================================
// App state
// =============================================================================

/// Identifiable motor on the CAN bus.
#[derive(Debug, Clone)]
struct MotorEntry {
    /// Vendor + CAN ID + model packed into the cross-vendor MotorSpec.
    spec: MotorSpec,
    enabled: bool,
    feedback: Option<MotorFeedback>,
    last_update: Option<Instant>,
    uuid: Option<Vec<u8>>,
    error: Option<String>,
}

impl MotorEntry {
    fn id(&self) -> u8 {
        self.spec.can_id()
    }

    /// Short label for the Model column (e.g. "RS-05" or "DM-J4310").
    fn model_label(&self) -> String {
        match &self.spec {
            MotorSpec::Robstride { model, .. } => format!("{}", model),
            MotorSpec::Damiao { model, .. } => format!("{}", model),
        }
    }
}

/// Which UI panel currently has focus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Focus {
    Motors,
    Commands,
    Params,
    #[allow(dead_code)]
    Input,
}

/// A parameter field shown in the Params panel.
#[derive(Debug, Clone)]
struct ParamField {
    /// Display name
    name: &'static str,
    /// Current value as string
    value: String,
    /// Default value as string
    default: String,
    /// Description / hint
    desc: &'static str,
    /// Is this an enum-like choice? (list options separated by |)
    choices: Option<&'static str>,
}

impl ParamField {
    fn new(name: &'static str, value: &str, desc: &'static str) -> Self {
        ParamField {
            name,
            value: value.to_string(),
            default: value.to_string(),
            desc,
            choices: None,
        }
    }

    fn with_choices(name: &'static str, value: &str, desc: &'static str, choices: &'static str) -> Self {
        ParamField {
            name,
            value: value.to_string(),
            default: value.to_string(),
            desc,
            choices: Some(choices),
        }
    }
}

/// Get parameter fields for the selected command.
fn params_for_command(cmd: Command) -> Vec<ParamField> {
    match cmd {
        Command::Scan => vec![
            ParamField::new("from", "1", "Start ID [1-254]"),
            ParamField::new("to", "127", "End ID [1-254]"),
            ParamField::with_choices("vendor", "rs", "Vendor to probe (rs=Robstride, dm=DAMIAO, both)",
                "rs|dm|both"),
        ],
        Command::ReadParam => vec![
            ParamField::with_choices("param", "mech_pos", "Parameter name",
                "mech_pos|mech_vel|iq_filt|vbus|limit_torque|limit_spd|limit_cur|run_mode|loc_kp|spd_kp|spd_ki"),
        ],
        Command::WriteParam => vec![
            ParamField::with_choices("param", "limit_spd", "Parameter name",
                "limit_torque|limit_spd|limit_cur|loc_kp|spd_kp|spd_ki"),
            ParamField::new("value", "10.0", "Float value to write"),
        ],
        Command::SetRunMode => vec![
            ParamField::with_choices("mode", "mit", "Motor run mode",
                "mit|position|velocity|torque"),
        ],
        Command::MoveTo => vec![
            ParamField::new("position", "0.0", "Target position [rad]"),
            ParamField::new("speed", "5.0", "Speed limit [rad/s]"),
        ],
        Command::Spin => vec![
            ParamField::new("velocity", "1.0", "Target velocity [rad/s]"),
        ],
        Command::Torque => vec![
            ParamField::new("torque", "0.0", "Target torque [Nm]"),
        ],
        Command::MitControl => vec![
            ParamField::new("pos", "0.0", "Position ref [rad]"),
            ParamField::new("vel", "0.0", "Velocity ref [rad/s]"),
            ParamField::new("kp", "10.0", "Proportional gain"),
            ParamField::new("kd", "0.5", "Derivative gain"),
            ParamField::new("torque", "0.0", "Torque FF [Nm]"),
        ],
        Command::Bilateral => vec![
            ParamField::with_choices("lead_kind", "rs05", "Leader motor kind",
                MOTOR_KIND_CHOICES),
            ParamField::new("lead_id", "10", "Leader CAN ID"),
            ParamField::with_choices("lead_inv", "0", "Leader polarity flip (1 to invert sign)",
                "0|1"),
            ParamField::with_choices("foll_kind", "rs05", "Follower motor kind",
                MOTOR_KIND_CHOICES),
            ParamField::new("foll_id", "1", "Follower CAN ID"),
            ParamField::with_choices("foll_inv", "0", "Follower polarity flip (1 to invert sign)",
                "0|1"),
            ParamField::with_choices("method", "coupling", "Control method",
                "pos|force|coupling|mode|ondemand|ondemand_emu|coupling_mit"),
            ParamField::new("kp", "5.0", "Spring stiffness [Nm/rad]"),
            ParamField::new("kd", "0.3", "Damping [Nm·s/rad]"),
            ParamField::new("coulomb", "0.05", "Coulomb friction comp [Nm]"),
            ParamField::new("viscous", "0.01", "Viscous friction comp [Nm·s/rad]"),
            ParamField::new("force_sc", "0.5", "Force refl scale (force/ondemand)"),
            ParamField::new("inertia", "0.005", "Motor inertia [kg·m²]"),
            ParamField::new("dob_cut", "100.0", "DOB cutoff [rad/s] (mode method)"),
            ParamField::new("iner_comp", "0.0", "Leader inertia FF comp [0-1]"),
            ParamField::new("acc_cut", "50.0", "Accel LPF cutoff [rad/s]"),
            ParamField::new("assist_kd", "0.0", "Leader motor-internal kd assist (0=off; CAUTION: low-friction motors)"),
            ParamField::new("vel_ahead", "2.0", "Vel ref lookahead (1=off, 2-3 typ)"),
            ParamField::new("max_assist", "0.05", "Max assist torque [Nm] (safety limit)"),
            ParamField::new("f_thresh", "0.3", "Force threshold [Nm] (ondemand mode)"),
            ParamField::with_choices("open_sign", "0", "Opening dir (ondemand: 0=off, +1/-1)",
                "-1|0|+1"),
            ParamField::new("safety_rad", "3.14", "Disable both motors if |pos|>this [rad] (0=off)"),
            ParamField::new("safety_jump", "0.5", "Disable if Δpos/cycle > this [rad] (0=off)"),
            ParamField::new("vel_cut", "0.0", "Velocity LPF cutoff [rad/s] for kd (0=raw motor vel, recommended for DAMIAO)"),
            ParamField::new("tau_slew", "0.0", "Torque slew limit [Nm/cycle] (0=off)"),
        ],
        Command::ZeroPair => vec![
            ParamField::with_choices("lead_kind", "rs05", "Leader motor kind",
                MOTOR_KIND_CHOICES),
            ParamField::new("lead_id", "10", "Leader CAN ID"),
            ParamField::with_choices("foll_kind", "rs05", "Follower motor kind",
                MOTOR_KIND_CHOICES),
            ParamField::new("foll_id", "1", "Follower CAN ID"),
        ],
        Command::AssistTest => vec![
            ParamField::with_choices("motor_kind", "rs05", "Motor kind", MOTOR_KIND_CHOICES),
            ParamField::new("motor_id", "10", "Motor CAN ID to test"),
            ParamField::new("assist_kd", "0.0", "Motor-internal kd assist [Nm·s/rad] (0=off)"),
            ParamField::new("vel_ahead", "2.0", "Vel ref lookahead (1=off, 2-3 typ)"),
            ParamField::new("max_assist", "0.05", "Max assist torque [Nm] (safety limit)"),
            ParamField::new("coulomb", "0.0", "Coulomb friction comp [Nm] (0=off)"),
            ParamField::new("viscous", "0.0", "Viscous friction comp [Nm·s/rad] (0=off)"),
            ParamField::new("inertia", "0.005", "Motor inertia [kg·m²]"),
            ParamField::new("iner_comp", "0.0", "Inertia FF comp ratio [0-1] (0=off)"),
            ParamField::new("acc_cut", "50.0", "Accel LPF cutoff [rad/s]"),
        ],
        // Commands with no parameters
        _ => vec![],
    }
}

/// Available commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Command {
    Scan,
    Ping,
    Enable,
    Disable,
    SetZero,
    ReadStatus,
    ReadParam,
    WriteParam,
    SetRunMode,
    MoveTo,
    Spin,
    Torque,
    MitControl,
    Bilateral,
    ZeroPair,
    AssistTest,
}

impl Command {
    const ALL: [Command; 16] = [
        Command::Scan,
        Command::Ping,
        Command::Enable,
        Command::Disable,
        Command::SetZero,
        Command::ReadStatus,
        Command::ReadParam,
        Command::WriteParam,
        Command::SetRunMode,
        Command::MoveTo,
        Command::Spin,
        Command::Torque,
        Command::MitControl,
        Command::Bilateral,
        Command::ZeroPair,
        Command::AssistTest,
    ];

    fn label(&self) -> &'static str {
        match self {
            Command::Scan => "Scan Bus",
            Command::Ping => "Ping",
            Command::Enable => "Enable",
            Command::Disable => "Disable",
            Command::SetZero => "Set Zero",
            Command::ReadStatus => "Read Status",
            Command::ReadParam => "Read Param",
            Command::WriteParam => "Write Param",
            Command::SetRunMode => "Set Run Mode",
            Command::MoveTo => "Move To Pos",
            Command::Spin => "Spin (Vel)",
            Command::Torque => "Torque",
            Command::MitControl => "MIT Control",
            Command::Bilateral => "Bilateral Ctrl",
            Command::ZeroPair => "Zero Pair",
            Command::AssistTest => "Assist Test",
        }
    }

    fn has_params(&self) -> bool {
        !params_for_command(*self).is_empty()
    }
}

struct App {
    /// CAN interface name
    interface: String,
    /// Default host ID
    host_id: u8,
    /// Default motor model
    default_model: MotorModel,
    /// List of discovered motors
    motors: Vec<MotorEntry>,
    /// Currently selected motor index
    selected_motor: usize,
    /// Which panel has focus
    focus: Focus,
    /// Command list selection
    selected_cmd: usize,
    /// Log messages
    log: Vec<String>,
    /// Maximum log entries
    log_max: usize,
    /// Vertical scroll offset for log
    log_scroll: u16,
    /// Input buffer (for commands requiring parameters)
    input_buf: String,
    /// Whether we're waiting for input
    input_mode: bool,
    /// The command that needs input
    pending_cmd: Option<Command>,
    /// Quit flag
    quit: bool,
    /// Auto-refresh interval for motor status (ms)
    refresh_interval_ms: u64,
    /// Last refresh
    last_refresh: Instant,
    /// Scan progress: (current, total, scanning_flag)
    scan_progress: Arc<Mutex<(usize, usize, bool)>>,
    /// Scan results collected from background thread
    scan_results: Arc<Mutex<Vec<(u8, Option<Vec<u8>>)>>>,
    /// Bilateral control telemetry (if running)
    bilateral_telemetry: Option<SharedTelemetry>,
    /// Bilateral control stop flag (if running)
    bilateral_stop: Option<StopFlag>,
    /// Parameter fields for the currently selected command
    params: Vec<ParamField>,
    /// Selected param index in the Params panel
    selected_param: usize,
    /// Whether we're editing a param value inline
    editing_param: bool,
    /// Edit buffer for inline param editing
    param_edit_buf: String,
    /// Persistent user config (saved to disk)
    config: AppConfig,
    /// Soft-zero offsets, keyed by MotorSpec::key().
    /// Set by Zero Pair, applied at Bilateral / Assist Test launch.
    /// Lives in-memory only — never written to motor NVM or to disk.
    soft_zero_offsets: HashMap<String, f64>,
    /// If `vendor=both` was selected, the DAMIAO scan to run synchronously
    /// once the (async) Robstride scan finishes. Cleared after firing.
    dm_scan_after: Option<(u8, u8)>,
}

impl App {
    fn new(interface: &str, host_id: u8, model: MotorModel) -> Self {
        let config = AppConfig::load();
        let initial_cmd = Command::ALL[0];
        let params = Self::apply_config_to_params(
            params_for_command(initial_cmd),
            initial_cmd.label(),
            &config,
        );
        App {
            interface: interface.to_string(),
            host_id,
            default_model: model,
            motors: Vec::new(),
            selected_motor: 0,
            focus: Focus::Commands,
            selected_cmd: 0,
            log: vec!["Robstride TUI started. Press Tab to switch panels.".to_string()],
            log_max: 500,
            log_scroll: 0,
            input_buf: String::new(),
            input_mode: false,
            pending_cmd: None,
            quit: false,
            refresh_interval_ms: 200,
            last_refresh: Instant::now(),
            scan_progress: Arc::new(Mutex::new((0, 0, false))),
            scan_results: Arc::new(Mutex::new(Vec::new())),
            bilateral_telemetry: None,
            bilateral_stop: None,
            params,
            selected_param: 0,
            editing_param: false,
            param_edit_buf: String::new(),
            config,
            soft_zero_offsets: HashMap::new(),
            dm_scan_after: None,
        }
    }

    fn log_msg(&mut self, msg: String) {
        let ts = chrono_like_timestamp();
        self.log.push(format!("[{}] {}", ts, msg));
        if self.log.len() > self.log_max {
            self.log.remove(0);
        }
        // Auto-scroll to bottom
        self.log_scroll = self.log.len().saturating_sub(1) as u16;
    }

    fn selected_motor_entry(&self) -> Option<&MotorEntry> {
        self.motors.get(self.selected_motor)
    }

    fn selected_motor_id(&self) -> Option<u8> {
        self.selected_motor_entry().map(|m| m.id())
    }

    // =========================================================================
    // Command execution
    // =========================================================================

    fn execute_scan(&mut self, input: &str) {
        // Check if already scanning
        let already_scanning = {
            let progress = self.scan_progress.lock().unwrap();
            progress.2
        };
        if already_scanning {
            self.log_msg("Scan already in progress.".to_string());
            return;
        }

        // Parse range + vendor from input. Params: from to vendor
        let parts: Vec<&str> = input.trim().split_whitespace().collect();
        let from: u8 = parts.first().and_then(|s| s.parse().ok()).unwrap_or(1);
        let to: u8 = parts.get(1).and_then(|s| s.parse().ok()).unwrap_or(127);
        let vendor = parts.get(2).copied().unwrap_or("rs").to_lowercase();
        let from = from.max(1);
        let to = to.max(from).min(254);

        let do_rs = vendor == "rs" || vendor == "both";
        let do_dm = vendor == "dm" || vendor == "both";
        if !do_rs && !do_dm {
            self.log_msg(format!(
                "Unknown vendor '{}'. Use rs, dm, or both.",
                vendor
            ));
            return;
        }

        // Robstride scan runs asynchronously (slow per-ID timeout); DAMIAO
        // scan is short (~10ms per ID, ~1 s for 1..127) so it runs inline
        // after the async scan completes (via self.dm_scan_after).
        if do_rs {
            self.log_msg(format!(
                "Scanning CAN bus (Robstride) ID {}..={}...",
                from, to
            ));
            {
                let mut progress = self.scan_progress.lock().unwrap();
                *progress = (0, (to - from + 1) as usize, true);
            }
            {
                let mut results = self.scan_results.lock().unwrap();
                results.clear();
            }
            let interface = self.interface.clone();
            let host_id = self.host_id;
            let progress = Arc::clone(&self.scan_progress);
            let results_out = Arc::clone(&self.scan_results);
            std::thread::spawn(move || {
                let results = Motor::scan_bus_progressive(
                    &interface,
                    host_id,
                    from..=to,
                    Duration::from_millis(100),
                    |current, total, _motor_id| {
                        if let Ok(mut p) = progress.lock() {
                            p.0 = current;
                            p.1 = total;
                        }
                    },
                );
                if let Ok(mut out) = results_out.lock() {
                    *out = results;
                }
                if let Ok(mut p) = progress.lock() {
                    p.2 = false;
                }
            });
            // Queue the DM scan to run when the Robstride scan finishes.
            self.dm_scan_after = if do_dm { Some((from, to)) } else { None };
        } else if do_dm {
            // DM-only scan: run inline (synchronous, ~1 s).
            self.run_damiao_scan(from, to);
        }
    }

    /// Synchronous DAMIAO scan probe. Logs found IDs and adds them to the
    /// Motors panel as DAMIAO entries.
    fn run_damiao_scan(&mut self, from: u8, to: u8) {
        self.log_msg(format!(
            "Scanning CAN bus (DAMIAO) ID {}..={} (probe takes ~10 ms/ID)...",
            from, to
        ));
        match driver::scan_damiao(&self.interface, from..=to, Duration::from_millis(10)) {
            Ok(ids) => {
                if ids.is_empty() {
                    self.log_msg("No DAMIAO motors found.".to_string());
                } else {
                    let pretty: Vec<String> =
                        ids.iter().map(|i| format!("{} (0x{:02X})", i, i)).collect();
                    self.log_msg(format!(
                        "Found {} DAMIAO motor(s): {}",
                        ids.len(),
                        pretty.join(", ")
                    ));
                    for id in ids {
                        let spec = MotorSpec::damiao(id, 0, DamiaoModel::DmJ4310_2EC);
                        let already_listed = self
                            .motors
                            .iter()
                            .any(|m| matches!(m.spec, MotorSpec::Damiao { .. }) && m.id() == id);
                        if !already_listed {
                            self.motors.push(MotorEntry {
                                spec,
                                enabled: false,
                                feedback: None,
                                last_update: None,
                                uuid: None,
                                error: None,
                            });
                        }
                    }
                    self.motors.sort_by_key(|m| m.id());
                }
            }
            Err(e) => self.log_msg(format!("DAMIAO scan error: {}", e)),
        }
    }

    /// Check if a background scan has completed and process results.
    fn check_scan_complete(&mut self) {
        let done = {
            let progress = self.scan_progress.lock().unwrap();
            !progress.2 && progress.0 > 0 && progress.0 == progress.1
        };
        if !done {
            return;
        }

        // Consume results
        let results: Vec<(u8, Option<Vec<u8>>)> = {
            let mut r = self.scan_results.lock().unwrap();
            std::mem::take(&mut *r)
        };
        // Reset progress
        {
            let mut p = self.scan_progress.lock().unwrap();
            *p = (0, 0, false);
        }

        if results.is_empty() {
            self.log_msg("No motors found.".to_string());
        } else {
            self.log_msg(format!("Found {} motor(s).", results.len()));
            for (id, data) in results {
                // Vendor-specific dedup: a DAMIAO at the same numeric ID
                // must not block adding the Robstride at that ID, and vice
                // versa. Robstride uses 29-bit extended IDs and DAMIAO uses
                // 11-bit standard IDs so the two share number space without
                // colliding on the wire.
                let exists = self.motors.iter().any(|m| {
                    matches!(m.spec, MotorSpec::Robstride { .. }) && m.id() == id
                });
                if !exists {
                    let entry = MotorEntry {
                        spec: MotorSpec::robstride(self.host_id, id, self.default_model),
                        enabled: false,
                        feedback: None,
                        last_update: None,
                        uuid: data.clone(),
                        error: None,
                    };
                    self.log_msg(format!(
                        "  Motor ID={} discovered (data={:?})",
                        id,
                        data.as_ref().map(|d| hex_str(d))
                    ));
                    self.motors.push(entry);
                }
            }
            self.motors.sort_by_key(|m| m.id());
        }

        // If "both" was requested, the DAMIAO scan was queued to follow.
        if let Some((from, to)) = self.dm_scan_after.take() {
            self.run_damiao_scan(from, to);
        }
    }

    /// Get scan progress ratio (0.0 - 1.0) and whether scanning is active.
    fn scan_state(&self) -> (f64, bool) {
        let p = self.scan_progress.lock().unwrap();
        let ratio = if p.1 > 0 {
            p.0 as f64 / p.1 as f64
        } else {
            0.0
        };
        (ratio, p.2)
    }

    fn execute_ping(&mut self) {
        if self.selected_motor_id().is_none() {
            self.log_msg("No motor selected.".to_string());
            return;
        }
        let idx = self.selected_motor;
        let spec = self.motors[idx].spec.clone();

        match spec {
            MotorSpec::Robstride {
                host_id,
                can_id,
                model,
                ..
            } => match Motor::new(&self.interface, can_id, host_id, model) {
                Ok(motor) => match motor.ping() {
                    Ok((device_id, uuid)) => {
                        self.log_msg(format!(
                            "Ping OK: motor={} device_id=0x{:04X} UUID=[{}]",
                            can_id,
                            device_id,
                            hex_str(&uuid)
                        ));
                        self.motors[idx].uuid = Some(uuid);
                    }
                    Err(e) => self.log_msg(format!("Ping failed: {}", e)),
                },
                Err(e) => self.log_msg(format!("CAN open error: {}", e)),
            },
            MotorSpec::Damiao { .. } => {
                let socket = match CanSocket::open(&self.interface) {
                    Ok(s) => s,
                    Err(e) => {
                        self.log_msg(format!("CAN open error: {}", e));
                        return;
                    }
                };
                let _ = socket.set_read_timeout(Duration::from_millis(50));
                let mut driver = self.apply_saved_offset(spec).build();
                match driver.enable(&socket) {
                    Ok(()) => {
                        match driver.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0) {
                            Ok(fb) => {
                                self.log_msg(format!(
                                    "Ping OK (DAMIAO): pos={:.4} vel={:.4} torque={:.4}",
                                    fb.position, fb.velocity, fb.torque
                                ));
                                self.motors[idx].feedback = Some(fb);
                                self.motors[idx].last_update = Some(Instant::now());
                                self.motors[idx].error = None;
                            }
                            Err(e) => self.log_msg(format!("Ping failed: {}", e)),
                        }
                        let _ = driver.disable(&socket);
                    }
                    Err(e) => self.log_msg(format!("Ping failed: {}", e)),
                }
            }
        }
    }

    /// Vendor-agnostic enable via the MotorDriver trait. After enabling, sends
    /// a zero MIT exchange to capture the current state (position/velocity)
    /// so the panel shows the angle even for DAMIAO motors.
    fn execute_enable(&mut self) {
        if self.selected_motor_id().is_none() {
            self.log_msg("No motor selected.".to_string());
            return;
        }
        let idx = self.selected_motor;
        let spec = self.motors[idx].spec.clone();
        let label = spec.description();
        let socket = match CanSocket::open(&self.interface) {
            Ok(s) => s,
            Err(e) => {
                self.log_msg(format!("CAN open error: {}", e));
                return;
            }
        };
        let _ = socket.set_read_timeout(Duration::from_millis(50));
        let mut driver = self.apply_saved_offset(spec).build();
        match driver.enable(&socket) {
            Ok(()) => {
                self.motors[idx].enabled = true;
                self.log_msg(format!("{} enabled.", label));
                // Read current state with a zero MIT command (no torque).
                if let Ok(fb) = driver.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0) {
                    self.motors[idx].feedback = Some(fb);
                    self.motors[idx].last_update = Some(Instant::now());
                    self.motors[idx].error = None;
                }
            }
            Err(e) => self.log_msg(format!("Enable failed: {}", e)),
        }
    }

    fn execute_disable(&mut self) {
        if self.selected_motor_id().is_none() {
            self.log_msg("No motor selected.".to_string());
            return;
        }
        let idx = self.selected_motor;
        let spec = self.motors[idx].spec.clone();
        let label = spec.description();
        let socket = match CanSocket::open(&self.interface) {
            Ok(s) => s,
            Err(e) => {
                self.log_msg(format!("CAN open error: {}", e));
                return;
            }
        };
        let _ = socket.set_read_timeout(Duration::from_millis(50));
        let mut driver = self.apply_saved_offset(spec).build();
        match driver.disable(&socket) {
            Ok(()) => {
                self.motors[idx].enabled = false;
                self.log_msg(format!("{} disabled.", label));
            }
            Err(e) => self.log_msg(format!("Disable failed: {}", e)),
        }
    }

    /// Persist the current physical position as the motor's hardware zero
    /// (NVM write). Both vendors are supported. For routine recalibration
    /// prefer Zero Pair (in-memory) — NVM has finite write endurance.
    ///
    /// After a successful NVM write, the raw-frame soft_zero offset cached
    /// in the App for this motor is also cleared, because the motor's
    /// reported raw=0 is now this physical pose.
    fn execute_set_zero(&mut self) {
        if self.selected_motor_id().is_none() {
            self.log_msg("No motor selected.".to_string());
            return;
        }
        let idx = self.selected_motor;
        let spec = self.motors[idx].spec.clone();
        let label = spec.description();
        let key = spec.key();

        self.log_msg(format!(
            "WARNING: writing zero to NVM for {} (flash wear — use Zero Pair for routine recalibration)",
            label
        ));

        let outcome = match &spec {
            MotorSpec::Robstride {
                host_id,
                can_id,
                model,
                ..
            } => match Motor::new(&self.interface, *can_id, *host_id, *model) {
                Ok(mut motor) => motor
                    .set_zero()
                    .map_err(|e| format!("Set zero failed: {}", e)),
                Err(e) => Err(format!("CAN open error: {}", e)),
            },
            MotorSpec::Damiao { can_id, .. } => match CanSocket::open(&self.interface) {
                Ok(socket) => {
                    let _ = socket.set_read_timeout(Duration::from_millis(50));
                    driver::damiao_set_zero_nvm(&socket, *can_id)
                        .map_err(|e| format!("Set zero failed: {}", e))
                }
                Err(e) => Err(format!("CAN open error: {}", e)),
            },
        };

        match outcome {
            Ok(()) => {
                self.log_msg(format!("{} zero saved to NVM.", label));
                // Hardware zero changes the motor's raw frame, so any cached
                // soft_zero (in raw frame) is now invalid — drop it.
                if self.soft_zero_offsets.remove(&key).is_some() {
                    self.log_msg(format!(
                        "  Cleared cached soft_zero for {} (no longer needed).",
                        label
                    ));
                }
                // The previous feedback reading also reflects the old frame.
                self.motors[idx].feedback = None;
            }
            Err(msg) => self.log_msg(msg),
        }
    }

    fn execute_read_status(&mut self) {
        if self.selected_motor_id().is_none() {
            self.log_msg("No motor selected.".to_string());
            return;
        }
        let idx = self.selected_motor;
        let spec = self.motors[idx].spec.clone();
        let was_enabled = self.motors[idx].enabled;
        let socket = match CanSocket::open(&self.interface) {
            Ok(s) => s,
            Err(e) => {
                self.log_msg(format!("CAN open error: {}", e));
                return;
            }
        };
        let _ = socket.set_read_timeout(Duration::from_millis(50));
        let mut driver = self.apply_saved_offset(spec.clone()).build();

        // DAMIAO MIT responses require an enabled motor; if the user hasn't
        // enabled it, surface that clearly rather than time out.
        let is_dm = matches!(spec, MotorSpec::Damiao { .. });
        if is_dm && !was_enabled {
            self.log_msg(
                "DAMIAO Read Status needs the motor enabled (DM MIT mode does \
                 not respond while disabled). Run Enable first."
                    .to_string(),
            );
            return;
        }

        match driver.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0) {
            Ok(fb) => {
                self.log_msg(format!(
                    "Status: pos={:.4} vel={:.4} torque={:.4} temp={:.1}°C mode={}",
                    fb.position, fb.velocity, fb.torque, fb.temperature, fb.status.mode
                ));
                self.motors[idx].feedback = Some(fb);
                self.motors[idx].last_update = Some(Instant::now());
                self.motors[idx].error = None;
            }
            Err(e) => {
                self.log_msg(format!("Read status failed: {}", e));
                self.motors[idx].error = Some(e.to_string());
            }
        }
    }

    fn execute_read_param(&mut self, input: &str) {
        if self.selected_motor_id().is_none() {
            self.log_msg("No motor selected.".to_string());
            return;
        }
        let idx = self.selected_motor;
        let spec = self.motors[idx].spec.clone();
        let name = input.trim().to_lowercase();

        let param = match parse_param_name(&name) {
            Some(p) => p,
            None => {
                self.log_msg(format!("Unknown param: '{}'. Available: mech_pos, mech_vel, iq_filt, vbus, limit_torque, limit_spd, limit_cur, run_mode, loc_kp, spd_kp, spd_ki", input));
                return;
            }
        };

        match spec {
            MotorSpec::Robstride {
                host_id,
                can_id,
                model,
                ..
            } => match Motor::new(&self.interface, can_id, host_id, model) {
                Ok(motor) => match motor.read_param(param) {
                    Ok(val) => self.log_msg(format!("{} = {:.4}", input.trim(), val)),
                    Err(e) => self.log_msg(format!("Read param failed: {}", e)),
                },
                Err(e) => self.log_msg(format!("CAN open error: {}", e)),
            },
            MotorSpec::Damiao { .. } => {
                let socket = match CanSocket::open(&self.interface) {
                    Ok(s) => s,
                    Err(e) => {
                        self.log_msg(format!("CAN open error: {}", e));
                        return;
                    }
                };
                let _ = socket.set_read_timeout(Duration::from_millis(50));
                let mut driver = self.apply_saved_offset(spec).build();
                if let Err(e) = driver.enable(&socket) {
                    self.log_msg(format!("Read param failed: {}", e));
                    return;
                }
                let read_result = driver.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0);
                let _ = driver.disable(&socket);
                match read_result {
                    Ok(fb) => {
                        let out = match name.as_str() {
                            "mech_pos" | "position" => Some(fb.position),
                            "mech_vel" | "velocity" => Some(fb.velocity),
                            "run_mode" | "mode" => {
                                self.log_msg(format!("{} = mit", input.trim()));
                                None
                            }
                            _ => {
                                self.log_msg(format!(
                                    "DAMIAO read-param '{}' is not available in MIT-only mode",
                                    input.trim()
                                ));
                                None
                            }
                        };
                        if let Some(v) = out {
                            self.log_msg(format!("{} = {:.4}", input.trim(), v));
                        }
                    }
                    Err(e) => self.log_msg(format!("Read param failed: {}", e)),
                }
            }
        }
    }

    fn execute_write_param(&mut self, input: &str) {
        if self.selected_motor_id().is_none() {
            self.log_msg("No motor selected.".to_string());
            return;
        }
        let idx = self.selected_motor;
        let spec = self.motors[idx].spec.clone();

        let parts: Vec<&str> = input.trim().split_whitespace().collect();
        if parts.len() != 2 {
            self.log_msg("Usage: name value (e.g. limit_spd 10.0)".to_string());
            return;
        }
        let param = match parse_param_name(parts[0]) {
            Some(p) => p,
            None => {
                self.log_msg(format!("Unknown param: '{}'", parts[0]));
                return;
            }
        };
        let value: f32 = match parts[1].parse() {
            Ok(v) => v,
            Err(_) => {
                self.log_msg(format!("Invalid value: '{}'", parts[1]));
                return;
            }
        };

        match spec {
            MotorSpec::Robstride {
                host_id,
                can_id,
                model,
                ..
            } => match Motor::new(&self.interface, can_id, host_id, model) {
                Ok(motor) => match motor.write_param_f32(param, value) {
                    Ok(()) => self.log_msg(format!("{} = {:.4} (written)", parts[0], value)),
                    Err(e) => self.log_msg(format!("Write param failed: {}", e)),
                },
                Err(e) => self.log_msg(format!("CAN open error: {}", e)),
            },
            MotorSpec::Damiao { .. } => {
                self.log_msg(format!(
                    "DAMIAO write-param '{}' is not supported in MIT-only mode",
                    parts[0]
                ));
            }
        }
    }

    fn execute_set_run_mode(&mut self, input: &str) {
        if self.selected_motor_id().is_none() {
            self.log_msg("No motor selected.".to_string());
            return;
        }
        let idx = self.selected_motor;
        let spec = self.motors[idx].spec.clone();
        let mode = match input.trim().to_lowercase().as_str() {
            "mit" | "0" => RunMode::Mit,
            "position" | "pos" | "1" => RunMode::Position,
            "velocity" | "vel" | "2" => RunMode::Velocity,
            "torque" | "cur" | "3" => RunMode::Torque,
            _ => {
                self.log_msg("Unknown mode. Use: mit, position, velocity, torque".to_string());
                return;
            }
        };

        match spec {
            MotorSpec::Robstride {
                host_id,
                can_id,
                model,
                ..
            } => match Motor::new(&self.interface, can_id, host_id, model) {
                Ok(mut motor) => match motor.set_run_mode(mode) {
                    Ok(()) => self.log_msg(format!("Run mode set to {:?}", mode)),
                    Err(e) => self.log_msg(format!("Set run mode failed: {}", e)),
                },
                Err(e) => self.log_msg(format!("CAN open error: {}", e)),
            },
            MotorSpec::Damiao { .. } => {
                if matches!(mode, RunMode::Mit) {
                    self.log_msg("DAMIAO uses MIT mode only (already active).".to_string());
                } else {
                    self.log_msg("DAMIAO supports MIT mode only in this TUI.".to_string());
                }
            }
        }
    }

    fn execute_move_to(&mut self, input: &str) {
        if self.selected_motor_id().is_none() {
            self.log_msg("No motor selected.".to_string());
            return;
        }
        let idx = self.selected_motor;
        let spec = self.motors[idx].spec.clone();

        let parts: Vec<&str> = input.trim().split_whitespace().collect();
        if parts.is_empty() {
            self.log_msg("Usage: position_rad [speed_limit]".to_string());
            return;
        }
        let pos: f32 = match parts[0].parse() {
            Ok(v) => v,
            Err(_) => {
                self.log_msg("Invalid position value.".to_string());
                return;
            }
        };
        let speed: f32 = if parts.len() > 1 {
            parts[1].parse().unwrap_or(5.0)
        } else {
            5.0
        };

        match spec {
            MotorSpec::Robstride {
                host_id,
                can_id,
                model,
                ..
            } => match Motor::new(&self.interface, can_id, host_id, model) {
                Ok(mut motor) => {
                    let r = (|| -> std::result::Result<(), robstride_sandbox::error::RobstrideError> {
                        motor.disable()?;
                        motor.set_run_mode(RunMode::Position)?;
                        motor.set_position_speed_limit(speed)?;
                        motor.enable()?;
                        motor.set_position(pos)?;
                        Ok(())
                    })();
                    match r {
                        Ok(()) => {
                            self.log_msg(format!("Moving to {:.3} rad (speed limit={:.1})", pos, speed));
                            self.motors[idx].enabled = true;
                        }
                        Err(e) => self.log_msg(format!("Move to failed: {}", e)),
                    }
                    std::mem::forget(motor);
                }
                Err(e) => self.log_msg(format!("CAN open error: {}", e)),
            },
            MotorSpec::Damiao { .. } => {
                let socket = match CanSocket::open(&self.interface) {
                    Ok(s) => s,
                    Err(e) => {
                        self.log_msg(format!("CAN open error: {}", e));
                        return;
                    }
                };
                let _ = socket.set_read_timeout(Duration::from_millis(50));
                let mut driver = self.apply_saved_offset(spec).build();
                let r = (|| -> robstride_sandbox::error::Result<MotorFeedback> {
                    driver.enable(&socket)?;
                    driver.mit_exchange(&socket, pos as f64, 0.0, DM_MOVE_KP, DM_MOVE_KD, 0.0)
                })();
                match r {
                    Ok(fb) => {
                        self.log_msg(format!(
                            "DAMIAO move target sent: pos={:.3} (vel clamp={:.1})",
                            pos, speed
                        ));
                        self.motors[idx].enabled = true;
                        self.motors[idx].feedback = Some(fb);
                        self.motors[idx].last_update = Some(Instant::now());
                        self.motors[idx].error = None;
                    }
                    Err(e) => self.log_msg(format!("Move to failed: {}", e)),
                }
            }
        }
    }

    fn execute_spin(&mut self, input: &str) {
        if self.selected_motor_id().is_none() {
            self.log_msg("No motor selected.".to_string());
            return;
        }
        let idx = self.selected_motor;
        let spec = self.motors[idx].spec.clone();

        let vel: f32 = match input.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.log_msg("Invalid velocity value.".to_string());
                return;
            }
        };

        match spec {
            MotorSpec::Robstride {
                host_id,
                can_id,
                model,
                ..
            } => match Motor::new(&self.interface, can_id, host_id, model) {
                Ok(mut motor) => {
                    let r = (|| -> std::result::Result<(), robstride_sandbox::error::RobstrideError> {
                        motor.disable()?;
                        motor.set_run_mode(RunMode::Velocity)?;
                        motor.enable()?;
                        motor.set_velocity(vel)?;
                        Ok(())
                    })();
                    match r {
                        Ok(()) => {
                            self.log_msg(format!("Spinning at {:.2} rad/s", vel));
                            self.motors[idx].enabled = true;
                        }
                        Err(e) => self.log_msg(format!("Spin failed: {}", e)),
                    }
                    std::mem::forget(motor);
                }
                Err(e) => self.log_msg(format!("CAN open error: {}", e)),
            },
            MotorSpec::Damiao { .. } => {
                let socket = match CanSocket::open(&self.interface) {
                    Ok(s) => s,
                    Err(e) => {
                        self.log_msg(format!("CAN open error: {}", e));
                        return;
                    }
                };
                let _ = socket.set_read_timeout(Duration::from_millis(50));
                let mut driver = self.apply_saved_offset(spec).build();
                let r = (|| -> robstride_sandbox::error::Result<MotorFeedback> {
                    driver.enable(&socket)?;
                    driver.mit_exchange(&socket, 0.0, vel as f64, 0.0, DM_SPIN_KD, 0.0)
                })();
                match r {
                    Ok(fb) => {
                        self.log_msg(format!("DAMIAO velocity target sent: {:.2} rad/s", vel));
                        self.motors[idx].enabled = true;
                        self.motors[idx].feedback = Some(fb);
                        self.motors[idx].last_update = Some(Instant::now());
                        self.motors[idx].error = None;
                    }
                    Err(e) => self.log_msg(format!("Spin failed: {}", e)),
                }
            }
        }
    }

    fn execute_torque(&mut self, input: &str) {
        if self.selected_motor_id().is_none() {
            self.log_msg("No motor selected.".to_string());
            return;
        }
        let idx = self.selected_motor;
        let spec = self.motors[idx].spec.clone();

        let torque: f32 = match input.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.log_msg("Invalid torque value.".to_string());
                return;
            }
        };

        match spec {
            MotorSpec::Robstride {
                host_id,
                can_id,
                model,
                ..
            } => match Motor::new(&self.interface, can_id, host_id, model) {
                Ok(mut motor) => {
                    let r = (|| -> std::result::Result<(), robstride_sandbox::error::RobstrideError> {
                        motor.disable()?;
                        motor.set_run_mode(RunMode::Torque)?;
                        motor.enable()?;
                        motor.set_torque(torque)?;
                        Ok(())
                    })();
                    match r {
                        Ok(()) => {
                            self.log_msg(format!("Applying torque {:.3} Nm", torque));
                            self.motors[idx].enabled = true;
                        }
                        Err(e) => self.log_msg(format!("Torque failed: {}", e)),
                    }
                    std::mem::forget(motor);
                }
                Err(e) => self.log_msg(format!("CAN open error: {}", e)),
            },
            MotorSpec::Damiao { .. } => {
                let socket = match CanSocket::open(&self.interface) {
                    Ok(s) => s,
                    Err(e) => {
                        self.log_msg(format!("CAN open error: {}", e));
                        return;
                    }
                };
                let _ = socket.set_read_timeout(Duration::from_millis(50));
                let mut driver = self.apply_saved_offset(spec).build();
                let r = (|| -> robstride_sandbox::error::Result<MotorFeedback> {
                    driver.enable(&socket)?;
                    driver.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, torque as f64)
                })();
                match r {
                    Ok(fb) => {
                        self.log_msg(format!("DAMIAO torque target sent: {:.3} Nm", torque));
                        self.motors[idx].enabled = true;
                        self.motors[idx].feedback = Some(fb);
                        self.motors[idx].last_update = Some(Instant::now());
                        self.motors[idx].error = None;
                    }
                    Err(e) => self.log_msg(format!("Torque failed: {}", e)),
                }
            }
        }
    }

    fn execute_mit(&mut self, input: &str) {
        if self.selected_motor_id().is_none() {
            self.log_msg("No motor selected.".to_string());
            return;
        }
        let idx = self.selected_motor;
        let spec = self.motors[idx].spec.clone();

        let parts: Vec<f64> = input
            .trim()
            .split_whitespace()
            .filter_map(|s| s.parse().ok())
            .collect();
        if parts.len() != 5 {
            self.log_msg("Usage: pos vel kp kd torque (5 values)".to_string());
            return;
        }

        match spec {
            MotorSpec::Robstride {
                host_id,
                can_id,
                model,
                ..
            } => match Motor::new(&self.interface, can_id, host_id, model) {
                Ok(mut motor) => {
                    let r =
                        (|| -> std::result::Result<MotorFeedback, robstride_sandbox::error::RobstrideError> {
                            if !self.motors[idx].enabled {
                                motor.enable()?;
                                self.motors[idx].enabled = true;
                            }
                            motor.mit_control(parts[0], parts[1], parts[2], parts[3], parts[4])
                        })();
                    match r {
                        Ok(fb) => {
                            self.log_msg(format!(
                                "MIT: pos={:.4} vel={:.4} torque={:.4}",
                                fb.position, fb.velocity, fb.torque
                            ));
                            self.motors[idx].feedback = Some(fb);
                            self.motors[idx].last_update = Some(Instant::now());
                        }
                        Err(e) => self.log_msg(format!("MIT control failed: {}", e)),
                    }
                    std::mem::forget(motor);
                }
                Err(e) => self.log_msg(format!("CAN open error: {}", e)),
            },
            MotorSpec::Damiao { .. } => {
                let socket = match CanSocket::open(&self.interface) {
                    Ok(s) => s,
                    Err(e) => {
                        self.log_msg(format!("CAN open error: {}", e));
                        return;
                    }
                };
                let _ = socket.set_read_timeout(Duration::from_millis(50));
                let mut driver = self.apply_saved_offset(spec).build();
                let r = (|| -> robstride_sandbox::error::Result<MotorFeedback> {
                    if !self.motors[idx].enabled {
                        driver.enable(&socket)?;
                        self.motors[idx].enabled = true;
                    }
                    driver.mit_exchange(&socket, parts[0], parts[1], parts[2], parts[3], parts[4])
                })();
                match r {
                    Ok(fb) => {
                        self.log_msg(format!(
                            "MIT: pos={:.4} vel={:.4} torque={:.4}",
                            fb.position, fb.velocity, fb.torque
                        ));
                        self.motors[idx].feedback = Some(fb);
                        self.motors[idx].last_update = Some(Instant::now());
                        self.motors[idx].error = None;
                    }
                    Err(e) => self.log_msg(format!("MIT control failed: {}", e)),
                }
            }
        }
    }

    fn execute_assist_test(&mut self, input: &str) {
        // If already running, stop it
        if self.bilateral_stop.is_some() {
            self.stop_bilateral();
            return;
        }

        // Param order matches Command::AssistTest in params_for_command:
        //   0: motor_kind, 1: motor_id, 2: assist_kd, 3: vel_ahead,
        //   4: max_assist, 5: coulomb, 6: viscous, 7: inertia,
        //   8: inertia_comp, 9: accel_cutoff
        let parts: Vec<&str> = input.trim().split_whitespace().collect();
        let get = |i: usize| parts.get(i).copied();

        let (motor, motor_warn) = parse_motor_kind(
            get(0).unwrap_or("rs05"),
            get(1).unwrap_or("10"),
            10,
            self.host_id,
            self.default_model,
        );
        if let Some(w) = motor_warn { self.log_msg(w); }
        let motor = self.apply_saved_offset(motor);
        let mut cfg = AssistTestConfig {
            interface: self.interface.clone(),
            motor,
            ..Default::default()
        };
        if let Some(s) = get(2) { cfg.assist_kd = s.parse().unwrap_or(cfg.assist_kd); }
        if let Some(s) = get(3) { cfg.vel_ahead = s.parse().unwrap_or(cfg.vel_ahead); }
        if let Some(s) = get(4) { cfg.max_assist = s.parse().unwrap_or(cfg.max_assist); }
        if let Some(s) = get(5) { cfg.coulomb_friction = s.parse().unwrap_or(cfg.coulomb_friction); }
        if let Some(s) = get(6) { cfg.viscous_friction = s.parse().unwrap_or(cfg.viscous_friction); }
        if let Some(s) = get(7) { cfg.inertia = s.parse().unwrap_or(cfg.inertia); }
        if let Some(s) = get(8) { cfg.inertia_comp = s.parse().unwrap_or(cfg.inertia_comp); }
        if let Some(s) = get(9) { cfg.accel_cutoff = s.parse().unwrap_or(cfg.accel_cutoff); }

        self.log_msg(format!(
            "Starting Assist Test: {}, kd={:.2}, vel_ah={:.1}, maxA={:.3}, Cf={:.3}, Vf={:.3}, J={:.4}, IC={:.1}, AC={:.0}",
            cfg.motor.description(), cfg.assist_kd, cfg.vel_ahead, cfg.max_assist,
            cfg.coulomb_friction, cfg.viscous_friction,
            cfg.inertia, cfg.inertia_comp, cfg.accel_cutoff,
        ));
        self.log_msg("Press Esc to stop.".to_string());

        match bilateral::launch_assist_test(cfg) {
            Ok((telem, stop)) => {
                self.bilateral_telemetry = Some(telem);
                self.bilateral_stop = Some(stop);
            }
            Err(e) => {
                self.log_msg(format!("Assist test start failed: {}", e));
            }
        }
    }

    /// Zero both the leader and follower at their current physical positions.
    /// The zero is held in-memory inside the App (does NOT write to motor
    /// NVM) and re-applied when Bilateral / Assist Test is launched.
    /// Run with both joints held at the desired neutral pose; otherwise the
    /// initial position error will yank both motors.
    fn execute_zero_pair(&mut self, input: &str) {
        if self.bilateral_stop.is_some() {
            self.log_msg("Bilateral loop is running; stop it first.".to_string());
            return;
        }

        let parts: Vec<&str> = input.trim().split_whitespace().collect();
        let get = |i: usize| parts.get(i).copied();

        let (leader_spec, lw) = parse_motor_kind(
            get(0).unwrap_or("rs05"),
            get(1).unwrap_or("10"),
            10,
            self.host_id,
            self.default_model,
        );
        if let Some(w) = lw { self.log_msg(format!("Leader: {}", w)); }
        let (follower_spec, fw) = parse_motor_kind(
            get(2).unwrap_or("rs05"),
            get(3).unwrap_or("1"),
            1,
            self.host_id,
            self.default_model,
        );
        if let Some(w) = fw { self.log_msg(format!("Follower: {}", w)); }

        let socket = match CanSocket::open(&self.interface) {
            Ok(s) => s,
            Err(e) => {
                self.log_msg(format!("CAN open error: {}", e));
                return;
            }
        };
        if let Err(e) = socket.set_read_timeout(Duration::from_millis(100)) {
            self.log_msg(format!("Set read timeout failed: {}", e));
        }

        for spec in [&leader_spec, &follower_spec] {
            let mut drv = spec.build();
            let label = spec.description();
            match drv.set_soft_zero(&socket) {
                Ok(()) => {
                    let offset = drv.soft_zero_offset();
                    self.soft_zero_offsets.insert(spec.key(), offset);
                    self.log_msg(format!(
                        "Soft zero set: {} (offset = {:.4} rad, in-memory only)",
                        label, offset
                    ));
                }
                Err(e) => self.log_msg(format!("Soft zero {} failed: {}", label, e)),
            }
        }
    }

    /// Attach the saved soft-zero offset (if any) to `spec`.
    fn apply_saved_offset(&self, spec: MotorSpec) -> MotorSpec {
        match self.soft_zero_offsets.get(&spec.key()).copied() {
            Some(offset) => spec.with_soft_zero(offset),
            None => spec,
        }
    }

    fn execute_bilateral(&mut self, input: &str) {
        // If already running, stop it
        if self.bilateral_stop.is_some() {
            self.stop_bilateral();
            return;
        }

        // Param order matches Command::Bilateral in params_for_command:
        //   0: lead_kind, 1: lead_id, 2: lead_inv,
        //   3: foll_kind, 4: foll_id, 5: foll_inv,
        //   6: method, 7: kp, 8: kd, 9: coulomb, 10: viscous, 11: force_scale,
        //   12: inertia, 13: dob_cutoff, 14: inertia_comp, 15: accel_cutoff,
        //   16: assist_kd, 17: vel_ahead, 18: max_assist,
        //   19: force_threshold, 20: open_sign, 21: safety_rad, 22: safety_jump,
        //   23: vel_cut, 24: tau_slew
        let parts: Vec<&str> = input.trim().split_whitespace().collect();
        let get = |i: usize| parts.get(i).copied();
        let bool_flag = |s: &str| matches!(s.trim(), "1" | "true" | "yes" | "on");

        let lead_invert = get(2).map(bool_flag).unwrap_or(false);
        let (leader, leader_warn) = parse_motor_kind(
            get(0).unwrap_or("rs05"),
            get(1).unwrap_or("10"),
            10,
            self.host_id,
            self.default_model,
        );
        if let Some(w) = leader_warn { self.log_msg(format!("Leader: {}", w)); }
        let leader = self.apply_saved_offset(leader).with_invert(lead_invert);

        let foll_invert = get(5).map(bool_flag).unwrap_or(false);
        let (follower, follower_warn) = parse_motor_kind(
            get(3).unwrap_or("rs05"),
            get(4).unwrap_or("1"),
            1,
            self.host_id,
            self.default_model,
        );
        if let Some(w) = follower_warn { self.log_msg(format!("Follower: {}", w)); }
        let follower = self.apply_saved_offset(follower).with_invert(foll_invert);

        let method_str = get(6).unwrap_or("coupling");
        let method = match BilateralMethod::from_short(method_str) {
            Some(m) => m,
            None => {
                self.log_msg(format!(
                    "Unknown method '{}'. Use: pos, force, coupling, mode, ondemand, ondemand_emu, coupling_mit",
                    method_str
                ));
                return;
            }
        };

        let mut gains = BilateralGains::default();
        if let Some(s) = get(7) { gains.kp = s.parse().unwrap_or(gains.kp); }
        if let Some(s) = get(8) { gains.kd = s.parse().unwrap_or(gains.kd); }
        if let Some(s) = get(9) { gains.coulomb_friction = s.parse().unwrap_or(gains.coulomb_friction); }
        if let Some(s) = get(10) { gains.viscous_friction = s.parse().unwrap_or(gains.viscous_friction); }
        if let Some(s) = get(11) { gains.force_scale = s.parse().unwrap_or(gains.force_scale); }
        if let Some(s) = get(12) { gains.inertia = s.parse().unwrap_or(gains.inertia); }
        if let Some(s) = get(13) { gains.dob_cutoff = s.parse().unwrap_or(gains.dob_cutoff); }
        if let Some(s) = get(14) { gains.inertia_comp = s.parse().unwrap_or(gains.inertia_comp); }
        if let Some(s) = get(15) { gains.accel_cutoff = s.parse().unwrap_or(gains.accel_cutoff); }
        if let Some(s) = get(16) { gains.assist_kd = s.parse().unwrap_or(gains.assist_kd); }
        if let Some(s) = get(17) { gains.vel_ahead = s.parse().unwrap_or(gains.vel_ahead); }
        if let Some(s) = get(18) { gains.max_assist = s.parse().unwrap_or(gains.max_assist); }
        if let Some(s) = get(19) { gains.force_threshold = s.parse().unwrap_or(gains.force_threshold); }
        if let Some(s) = get(20) {
            // Parse +1, -1, 0 (strip leading '+')
            let trimmed = s.trim_start_matches('+');
            gains.open_sign = trimmed.parse().unwrap_or(gains.open_sign);
        }
        let safety_radius: f64 = get(21)
            .and_then(|s| s.parse().ok())
            .unwrap_or(std::f64::consts::PI);
        let safety_max_jump: f64 = get(22)
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.5);
        if let Some(s) = get(23) { gains.vel_cutoff = s.parse().unwrap_or(gains.vel_cutoff); }
        if let Some(s) = get(24) { gains.tau_slew = s.parse().unwrap_or(gains.tau_slew); }

        let config = BilateralConfig {
            interface: self.interface.clone(),
            leader,
            follower,
            method,
            ondemand: false,
            gains,
            loop_period_us: 2000,
            safety_radius,
            safety_max_jump,
        };

        self.log_msg(format!(
            "Starting bilateral control: {} (Kp={:.2}, Kd={:.2}, Cf={:.3}, Vf={:.3}, safety_rad={:.2}, safety_jump={:.2})",
            method.label(),
            gains.kp,
            gains.kd,
            gains.coulomb_friction,
            gains.viscous_friction,
            safety_radius,
            safety_max_jump,
        ));
        self.log_msg(format!(
            "  Leader={}, Follower={}  Press Esc to stop.",
            config.leader.description(),
            config.follower.description(),
        ));

        match bilateral::launch_bilateral(config) {
            Ok((telem, stop)) => {
                self.bilateral_telemetry = Some(telem);
                self.bilateral_stop = Some(stop);
            }
            Err(e) => {
                self.log_msg(format!("Bilateral start failed: {}", e));
            }
        }
    }

    fn stop_bilateral(&mut self) {
        if let Some(ref stop) = self.bilateral_stop {
            stop.store(true, std::sync::atomic::Ordering::Relaxed);
            self.log_msg("Bilateral control stopping...".to_string());
        }
        // Give the thread a moment to disable motors
        std::thread::sleep(Duration::from_millis(100));
        self.bilateral_telemetry = None;
        self.bilateral_stop = None;
        self.log_msg("Bilateral control stopped.".to_string());
    }

    /// Check if bilateral control is active.
    fn bilateral_active(&self) -> bool {
        self.bilateral_stop.is_some()
    }

    fn execute_command(&mut self, cmd: Command, input: &str) {
        match cmd {
            Command::Scan => self.execute_scan(input),
            Command::Ping => self.execute_ping(),
            Command::Enable => self.execute_enable(),
            Command::Disable => self.execute_disable(),
            Command::SetZero => self.execute_set_zero(),
            Command::ReadStatus => self.execute_read_status(),
            Command::ReadParam => self.execute_read_param(input),
            Command::WriteParam => self.execute_write_param(input),
            Command::SetRunMode => self.execute_set_run_mode(input),
            Command::MoveTo => self.execute_move_to(input),
            Command::Spin => self.execute_spin(input),
            Command::Torque => self.execute_torque(input),
            Command::MitControl => self.execute_mit(input),
            Command::Bilateral => self.execute_bilateral(input),
            Command::ZeroPair => self.execute_zero_pair(input),
            Command::AssistTest => self.execute_assist_test(input),
        }
    }

    /// Refresh status for all enabled motors via the MotorDriver trait. Works
    /// for both Robstride and DAMIAO; DM motors only refresh when enabled
    /// (DM MIT mode doesn't respond while disabled).
    fn refresh_motor_status(&mut self) {
        // Open one CanSocket and reuse for all enabled motors this tick.
        let socket = match CanSocket::open(&self.interface) {
            Ok(s) => s,
            Err(_) => return,
        };
        let _ = socket.set_read_timeout(Duration::from_millis(20));

        for i in 0..self.motors.len() {
            if !self.motors[i].enabled {
                continue;
            }
            let spec = self.motors[i].spec.clone();
            let mut driver = self.apply_saved_offset(spec).build();
            match driver.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0) {
                Ok(fb) => {
                    self.motors[i].feedback = Some(fb);
                    self.motors[i].last_update = Some(Instant::now());
                    self.motors[i].error = None;
                }
                Err(e) => {
                    self.motors[i].error = Some(e.to_string());
                }
            }
        }
    }

    // =========================================================================
    // Input handling
    // =========================================================================

    /// Build a space-separated input string from the current params.
    fn params_as_input(&self) -> String {
        self.params.iter().map(|p| p.value.as_str()).collect::<Vec<_>>().join(" ")
    }

    /// Sync params to the currently selected command (preserving values if names match).
    fn sync_params_to_cmd(&mut self) {
        let cmd = Command::ALL[self.selected_cmd];
        let new = params_for_command(cmd);
        // Try to preserve values from old params if same name
        let merged: Vec<ParamField> = new
            .into_iter()
            .map(|mut nf| {
                if let Some(old) = self.params.iter().find(|o| o.name == nf.name) {
                    nf.value = old.value.clone();
                }
                nf
            })
            .collect();
        // Apply saved config values on top
        self.params = Self::apply_config_to_params(merged, cmd.label(), &self.config);
        self.selected_param = 0;
    }

    /// Apply saved config values to param fields (overrides defaults).
    fn apply_config_to_params(
        params: Vec<ParamField>,
        cmd_label: &str,
        config: &AppConfig,
    ) -> Vec<ParamField> {
        params
            .into_iter()
            .map(|mut pf| {
                if let Some(saved) = config.get_param(cmd_label, pf.name) {
                    pf.value = saved.clone();
                }
                pf
            })
            .collect()
    }

    /// Save current params of the current command to config.
    fn save_current_params(&mut self) {
        let cmd = Command::ALL[self.selected_cmd];
        let label = cmd.label();
        for pf in &self.params {
            self.config.set_param(label, pf.name, &pf.value);
        }
        self.config.save();
    }

    fn handle_key(&mut self, key: KeyEvent) {
        // Global keys
        if key.code == KeyCode::Char('c') && key.modifiers.contains(KeyModifiers::CONTROL) {
            self.quit = true;
            return;
        }
        if key.code == KeyCode::Char('q') && !self.editing_param && !self.input_mode {
            if self.bilateral_active() {
                self.stop_bilateral();
                return;
            }
            self.quit = true;
            return;
        }

        // Esc stops bilateral control if running (outside editing)
        if key.code == KeyCode::Esc && !self.editing_param && !self.input_mode && self.bilateral_active() {
            self.stop_bilateral();
            return;
        }

        // Inline param editing mode
        if self.editing_param {
            match key.code {
                KeyCode::Enter => {
                    // Apply edited value
                    if self.selected_param < self.params.len() {
                        self.params[self.selected_param].value = self.param_edit_buf.clone();
                        self.save_current_params();
                    }
                    self.editing_param = false;
                    self.param_edit_buf.clear();
                }
                KeyCode::Esc => {
                    self.editing_param = false;
                    self.param_edit_buf.clear();
                }
                KeyCode::Char(c) => {
                    self.param_edit_buf.push(c);
                }
                KeyCode::Backspace => {
                    self.param_edit_buf.pop();
                }
                KeyCode::Tab => {
                    // Tab cycles through choices if available
                    if let Some(choices) = self.params.get(self.selected_param).and_then(|p| p.choices) {
                        let opts: Vec<&str> = choices.split('|').collect();
                        let cur = self.param_edit_buf.trim();
                        let idx = opts.iter().position(|o| *o == cur).unwrap_or(0);
                        let next = opts[(idx + 1) % opts.len()];
                        self.param_edit_buf = next.to_string();
                    }
                }
                _ => {}
            }
            return;
        }

        // Legacy input mode (still used for scan range etc. if needed)
        if self.input_mode {
            match key.code {
                KeyCode::Enter => {
                    let input = self.input_buf.clone();
                    self.input_mode = false;
                    if let Some(cmd) = self.pending_cmd.take() {
                        self.execute_command(cmd, &input);
                    }
                    self.input_buf.clear();
                    self.focus = Focus::Commands;
                }
                KeyCode::Esc => {
                    self.input_mode = false;
                    self.pending_cmd = None;
                    self.input_buf.clear();
                    self.focus = Focus::Commands;
                }
                KeyCode::Char(c) => {
                    self.input_buf.push(c);
                }
                KeyCode::Backspace => {
                    self.input_buf.pop();
                }
                _ => {}
            }
            return;
        }

        // Panel switching
        if key.code == KeyCode::Tab {
            let cmd = Command::ALL[self.selected_cmd];
            self.focus = match self.focus {
                Focus::Motors => Focus::Commands,
                Focus::Commands => {
                    if cmd.has_params() {
                        Focus::Params
                    } else {
                        Focus::Motors
                    }
                }
                Focus::Params => Focus::Motors,
                Focus::Input => Focus::Commands,
            };
            return;
        }
        // Shift+Tab = reverse
        if key.code == KeyCode::BackTab {
            let cmd = Command::ALL[self.selected_cmd];
            self.focus = match self.focus {
                Focus::Motors => {
                    if cmd.has_params() {
                        Focus::Params
                    } else {
                        Focus::Commands
                    }
                }
                Focus::Commands => Focus::Motors,
                Focus::Params => Focus::Commands,
                Focus::Input => Focus::Commands,
            };
            return;
        }

        match self.focus {
            Focus::Motors => match key.code {
                KeyCode::Up | KeyCode::Char('k') => {
                    if self.selected_motor > 0 {
                        self.selected_motor -= 1;
                    }
                }
                KeyCode::Down | KeyCode::Char('j') => {
                    if self.selected_motor + 1 < self.motors.len() {
                        self.selected_motor += 1;
                    }
                }
                KeyCode::Enter => {
                    // Quick read status on selected motor
                    self.execute_read_status();
                }
                _ => {}
            },
            Focus::Commands => match key.code {
                KeyCode::Up | KeyCode::Char('k') => {
                    if self.selected_cmd > 0 {
                        self.selected_cmd -= 1;
                        self.sync_params_to_cmd();
                    }
                }
                KeyCode::Down | KeyCode::Char('j') => {
                    if self.selected_cmd + 1 < Command::ALL.len() {
                        self.selected_cmd += 1;
                        self.sync_params_to_cmd();
                    }
                }
                KeyCode::Enter => {
                    let cmd = Command::ALL[self.selected_cmd];
                    let input = self.params_as_input();
                    self.execute_command(cmd, &input);
                }
                KeyCode::Right | KeyCode::Char('l') => {
                    let cmd = Command::ALL[self.selected_cmd];
                    if cmd.has_params() {
                        self.focus = Focus::Params;
                    }
                }
                _ => {}
            },
            Focus::Params => match key.code {
                KeyCode::Up | KeyCode::Char('k') => {
                    if self.selected_param > 0 {
                        self.selected_param -= 1;
                    }
                }
                KeyCode::Down | KeyCode::Char('j') => {
                    if self.selected_param + 1 < self.params.len() {
                        self.selected_param += 1;
                    }
                }
                KeyCode::Enter => {
                    // Start editing the selected param
                    if self.selected_param < self.params.len() {
                        self.editing_param = true;
                        self.param_edit_buf = self.params[self.selected_param].value.clone();
                    }
                }
                KeyCode::Left | KeyCode::Char('h') => {
                    self.focus = Focus::Commands;
                }
                // Quick execute with current params
                KeyCode::Char('x') => {
                    let cmd = Command::ALL[self.selected_cmd];
                    let input = self.params_as_input();
                    self.execute_command(cmd, &input);
                }
                _ => {}
            },
            Focus::Input => {}
        }
    }
}

// =============================================================================
// UI rendering
// =============================================================================

fn ui(frame: &mut Frame, app: &App) {
    // Overall layout: top (motors + commands + params) | bottom (log + input)
    let main_chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Percentage(45), Constraint::Percentage(55)])
        .split(frame.area());

    // Top area: motors (left) | commands (center) | params (right)
    let top_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage(45),
            Constraint::Percentage(18),
            Constraint::Percentage(37),
        ])
        .split(main_chunks[0]);

    // Bottom area: log (full width), possibly with input bar
    let bottom_chunks = if app.input_mode {
        Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Min(3), Constraint::Length(3)])
            .split(main_chunks[1])
    } else {
        Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Min(3), Constraint::Length(0)])
            .split(main_chunks[1])
    };

    render_motors(frame, app, top_chunks[0]);
    render_commands(frame, app, top_chunks[1]);
    render_params(frame, app, top_chunks[2]);
    render_log(frame, app, bottom_chunks[0]);
    if app.input_mode {
        render_input(frame, app, bottom_chunks[1]);
    }

    // Overlay bilateral telemetry if active
    if app.bilateral_active() {
        render_bilateral_overlay(frame, app);
    }

    // Overlay scan progress bar if scanning
    let (ratio, scanning) = app.scan_state();
    if scanning {
        let percent = (ratio * 100.0) as u16;
        let gauge = Gauge::default()
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(" Scanning ")
                    .border_style(Style::default().fg(Color::Cyan)),
            )
            .gauge_style(
                Style::default()
                    .fg(Color::Cyan)
                    .bg(Color::DarkGray)
                    .add_modifier(Modifier::BOLD),
            )
            .ratio(ratio.min(1.0))
            .label(format!("{}%", percent));

        // Place progress bar at bottom-center
        let area = frame.area();
        let bar_area = Rect {
            x: area.x + area.width / 6,
            y: area.y + area.height - 4,
            width: area.width * 2 / 3,
            height: 3,
        };
        // Clear background
        frame.render_widget(Clear, bar_area);
        frame.render_widget(gauge, bar_area);
    }
}

fn render_motors(frame: &mut Frame, app: &App, area: Rect) {
    let header = Row::new(vec![
        Cell::from("ID"),
        Cell::from("Model"),
        Cell::from("State"),
        Cell::from("Position"),
        Cell::from("Velocity"),
        Cell::from("Torque"),
        Cell::from("Temp"),
        Cell::from("Mode"),
    ])
    .style(Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD));

    let widths = [
        Constraint::Length(5),
        Constraint::Length(6),
        Constraint::Length(8),
        Constraint::Length(10),
        Constraint::Length(10),
        Constraint::Length(10),
        Constraint::Length(7),
        Constraint::Length(5),
    ];

    let rows: Vec<Row> = if app.motors.is_empty() {
        vec![Row::new(vec![Cell::from("(no motors – run Scan)")
            .style(Style::default().fg(Color::DarkGray))])]
    } else {
        app.motors
            .iter()
            .enumerate()
            .map(|(i, m)| {
                let state_str = if m.enabled { "ON" } else { "OFF" };
                let state_style = if m.enabled {
                    Style::default().fg(Color::Green)
                } else {
                    Style::default().fg(Color::DarkGray)
                };
                let (pos, vel, torque, temp, mode) = match &m.feedback {
                    Some(fb) => (
                        format!("{:>8.3}", fb.position),
                        format!("{:>8.3}", fb.velocity),
                        format!("{:>8.3}", fb.torque),
                        format!("{:>5.1}", fb.temperature),
                        format!("{}", fb.status.mode),
                    ),
                    None => (
                        "---".to_string(),
                        "---".to_string(),
                        "---".to_string(),
                        "---".to_string(),
                        "-".to_string(),
                    ),
                };
                let row_style = if i == app.selected_motor && app.focus == Focus::Motors {
                    Style::default().bg(Color::DarkGray)
                } else if i == app.selected_motor {
                    Style::default().add_modifier(Modifier::UNDERLINED)
                } else {
                    Style::default()
                };

                Row::new(vec![
                    Cell::from(format!("{}", m.id())),
                    Cell::from(m.model_label()),
                    Cell::from(state_str).style(state_style),
                    Cell::from(pos),
                    Cell::from(vel),
                    Cell::from(torque),
                    Cell::from(temp),
                    Cell::from(mode),
                ])
                .style(row_style)
            })
            .collect()
    };

    let border_style = if app.focus == Focus::Motors {
        Style::default().fg(Color::Cyan)
    } else {
        Style::default().fg(Color::White)
    };

    let table = Table::new(rows, widths)
        .header(header)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(" Motors ")
                .border_style(border_style),
        )
        .row_highlight_style(Style::default().bg(Color::DarkGray));

    frame.render_widget(table, area);
}

fn render_commands(frame: &mut Frame, app: &App, area: Rect) {
    let items: Vec<ListItem> = Command::ALL
        .iter()
        .enumerate()
        .map(|(i, cmd)| {
            let style = if i == app.selected_cmd && app.focus == Focus::Commands {
                Style::default()
                    .fg(Color::Black)
                    .bg(Color::Cyan)
                    .add_modifier(Modifier::BOLD)
            } else if i == app.selected_cmd {
                Style::default().fg(Color::Cyan)
            } else {
                Style::default()
            };
            ListItem::new(format!(" {} ", cmd.label())).style(style)
        })
        .collect();

    let border_style = if app.focus == Focus::Commands {
        Style::default().fg(Color::Cyan)
    } else {
        Style::default().fg(Color::White)
    };

    let list = List::new(items).block(
        Block::default()
            .borders(Borders::ALL)
            .title(" Commands ")
            .border_style(border_style),
    );

    frame.render_widget(list, area);
}

fn render_params(frame: &mut Frame, app: &App, area: Rect) {
    let cmd = Command::ALL[app.selected_cmd];
    let title = format!(" {} Params ", cmd.label());

    let border_style = if app.focus == Focus::Params {
        Style::default().fg(Color::Cyan)
    } else {
        Style::default().fg(Color::White)
    };

    if app.params.is_empty() {
        let block = Block::default()
            .borders(Borders::ALL)
            .title(title)
            .border_style(border_style);
        let p = Paragraph::new("  (no parameters)")
            .style(Style::default().fg(Color::DarkGray))
            .block(block);
        frame.render_widget(p, area);
        return;
    }

    let inner = area.inner(Margin { vertical: 1, horizontal: 1 });

    // Render the block border first
    let block = Block::default()
        .borders(Borders::ALL)
        .title(title)
        .border_style(border_style);
    frame.render_widget(block, area);

    // Render each parameter as a row
    let name_width = app.params.iter().map(|p| p.name.len()).max().unwrap_or(6).max(6);

    for (i, param) in app.params.iter().enumerate() {
        if i as u16 >= inner.height {
            break;
        }
        let row_area = Rect {
            x: inner.x,
            y: inner.y + i as u16,
            width: inner.width,
            height: 1,
        };

        let is_selected = i == app.selected_param && app.focus == Focus::Params;
        let is_editing = is_selected && app.editing_param;

        // Name part
        let name_style = if is_selected {
            Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::Yellow)
        };

        // Value part
        let (val_str, val_style) = if is_editing {
            // Show edit buffer with cursor
            (
                format!("{}_", app.param_edit_buf),
                Style::default().fg(Color::White).bg(Color::DarkGray),
            )
        } else {
            (
                param.value.clone(),
                if is_selected {
                    Style::default().fg(Color::White).add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(Color::White)
                },
            )
        };

        // Choice indicator + default value hint
        let default_hint = if param.value != param.default {
            format!(" (def:{})", param.default)
        } else {
            String::new()
        };
        let choice_marker = if param.choices.is_some() { "▼" } else { "" };

        let line = Line::from(vec![
            Span::styled(
                format!(" {:>width$} ", param.name, width = name_width),
                name_style,
            ),
            Span::styled(val_str, val_style),
            Span::styled(
                format!(" {}{}", choice_marker, default_hint),
                Style::default().fg(Color::DarkGray),
            ),
        ]);

        frame.render_widget(Paragraph::new(line), row_area);

        // Render description below for selected param
        if is_selected && !is_editing {
            // Show desc in the remaining space at bottom, or right after value
            let desc_y = inner.y + app.params.len().min(inner.height as usize) as u16;
            if desc_y < inner.y + inner.height {
                let desc_area = Rect {
                    x: inner.x,
                    y: desc_y,
                    width: inner.width,
                    height: 1,
                };
                let desc_line = Line::from(Span::styled(
                    format!(" → {}", param.desc),
                    Style::default().fg(Color::DarkGray).add_modifier(Modifier::ITALIC),
                ));
                frame.render_widget(Paragraph::new(desc_line), desc_area);

                // If choices, show them on next line
                if let Some(choices) = param.choices {
                    let choices_y = desc_y + 1;
                    if choices_y < inner.y + inner.height {
                        let choices_area = Rect {
                            x: inner.x,
                            y: choices_y,
                            width: inner.width,
                            height: 1,
                        };
                        let choices_line = Line::from(Span::styled(
                            format!("   [{}]", choices),
                            Style::default().fg(Color::DarkGray),
                        ));
                        frame.render_widget(Paragraph::new(choices_line), choices_area);
                    }
                }
            }
        }
    }
}

fn render_log(frame: &mut Frame, app: &App, area: Rect) {
    let inner_height = area.height.saturating_sub(2) as usize;
    let total = app.log.len();
    let start = if total > inner_height {
        total - inner_height
    } else {
        0
    };

    let items: Vec<ListItem> = app.log[start..]
        .iter()
        .map(|msg| {
            let style = if msg.contains("ERROR") || msg.contains("failed") {
                Style::default().fg(Color::Red)
            } else if msg.contains("OK") || msg.contains("enabled") || msg.contains("Found") {
                Style::default().fg(Color::Green)
            } else {
                Style::default().fg(Color::White)
            };
            ListItem::new(msg.as_str()).style(style)
        })
        .collect();

    let list = List::new(items).block(
        Block::default()
            .borders(Borders::ALL)
            .title(" Log ")
            .border_style(Style::default().fg(Color::White)),
    );

    frame.render_widget(list, area);
}

fn render_input(frame: &mut Frame, app: &App, area: Rect) {
    let title = format!(
        " Input: {} ",
        app.pending_cmd.map(|c| c.label()).unwrap_or("?"),
    );
    let paragraph = Paragraph::new(format!("{}_", app.input_buf))
        .block(
            Block::default()
                .borders(Borders::ALL)
                .title(title)
                .border_style(Style::default().fg(Color::Yellow)),
        )
        .style(Style::default().fg(Color::Yellow));

    frame.render_widget(paragraph, area);
}

fn render_bilateral_overlay(frame: &mut Frame, app: &App) {
    let telem = match &app.bilateral_telemetry {
        Some(t) => match t.lock() {
            Ok(t) => t.clone(),
            Err(_) => return,
        },
        None => return,
    };

    let is_assist_test = telem.method.is_none();
    let is_ondemand = matches!(
        telem.method,
        Some(BilateralMethod::OnDemand | BilateralMethod::EmulatedOnDemand)
    );
    let method_name = telem
        .method
        .map(|m| m.label())
        .unwrap_or("Assist Test");
    let overlay_title = if is_assist_test {
        " Assist Test "
    } else {
        " Bilateral Control "
    };

    let mut text = vec![
        Line::from(vec![
            Span::styled(" Mode: ", Style::default().fg(Color::Yellow)),
            Span::styled(method_name, Style::default().fg(Color::White).add_modifier(Modifier::BOLD)),
            Span::raw("    "),
            Span::styled(format!("Loop: {:.0} Hz", telem.loop_hz), Style::default().fg(Color::Cyan)),
            Span::raw("    "),
            Span::styled(format!("Cycles: {}", telem.cycle_count), Style::default().fg(Color::DarkGray)),
        ]),
    ];

    if is_ondemand {
        // OnDemand specific: leader ON/OFF status and detected force
        let leader_on = telem.leader_inertia_comp > 0.5;
        let detected_force = telem.leader_vel_assist; // reused field
        text.push(Line::from(vec![
            Span::styled(" Leader  (10): ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
            Span::raw(format!(
                "pos={:>8.3}  vel={:>8.3}  τcmd={:>7.3}  ",
                telem.leader_pos, telem.leader_vel, telem.leader_torque_cmd,
            )),
            Span::styled(
                if leader_on { "● ON " } else { "○ OFF" },
                if leader_on {
                    Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)
                } else {
                    Style::default().fg(Color::DarkGray)
                },
            ),
        ]));
        text.push(Line::from(vec![
            Span::styled(" Follow  ( 1): ", Style::default().fg(Color::Magenta).add_modifier(Modifier::BOLD)),
            Span::raw(format!(
                "pos={:>8.3}  vel={:>8.3}  τcmd={:>7.3}",
                telem.follower_pos, telem.follower_vel, telem.follower_torque_cmd,
            )),
        ]));
        text.push(Line::from(vec![
            Span::styled(" Δpos: ", Style::default().fg(Color::Yellow)),
            Span::styled(
                format!("{:>8.4} rad", telem.position_error),
                if telem.position_error.abs() > 0.5 {
                    Style::default().fg(Color::Red)
                } else {
                    Style::default().fg(Color::White)
                },
            ),
            Span::raw("   "),
            Span::styled("Force: ", Style::default().fg(Color::DarkGray)),
            Span::styled(
                format!("{:>6.3} Nm", detected_force),
                if leader_on {
                    Style::default().fg(Color::Yellow)
                } else {
                    Style::default().fg(Color::White)
                },
            ),
            Span::raw("  "),
            Span::styled("FrComp: ", Style::default().fg(Color::DarkGray)),
            Span::raw(format!("{:>6.3}", telem.follower_friction_comp)),
        ]));
    } else {
        // Regular bilateral or assist-test display
        text.push(Line::from(vec![
            Span::styled(
                if is_assist_test { " Motor:       " } else { " Leader  (10): " },
                Style::default().fg(Color::Green).add_modifier(Modifier::BOLD),
            ),
            Span::raw(format!(
                "pos={:>8.3}  vel={:>8.3}  τcmd={:>7.3}",
                telem.leader_pos, telem.leader_vel, telem.leader_torque_cmd,
            )),
        ]));

        if !is_assist_test {
            text.push(Line::from(vec![
                Span::styled(" Follow  ( 1): ", Style::default().fg(Color::Magenta).add_modifier(Modifier::BOLD)),
                Span::raw(format!(
                    "pos={:>8.3}  vel={:>8.3}  τcmd={:>7.3}",
                    telem.follower_pos, telem.follower_vel, telem.follower_torque_cmd,
                )),
            ]));
            text.push(Line::from(vec![
                Span::styled(" Δpos: ", Style::default().fg(Color::Yellow)),
                Span::styled(
                    format!("{:>8.4} rad", telem.position_error),
                    if telem.position_error.abs() > 0.5 {
                        Style::default().fg(Color::Red)
                    } else {
                        Style::default().fg(Color::White)
                    },
                ),
                Span::raw("   "),
                Span::styled("FrComp: ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!(
                    "L={:>6.3} F={:>6.3}",
                    telem.leader_friction_comp, telem.follower_friction_comp,
                )),
                Span::raw("  "),
                Span::styled("InrComp: ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:>6.3}", telem.leader_inertia_comp)),
                Span::raw("  "),
                Span::styled("VelAst: ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:>6.3}", telem.leader_vel_assist)),
            ]));
        } else {
            text.push(Line::from(vec![
                Span::styled(" FrComp: ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:>6.3}", telem.leader_friction_comp)),
                Span::raw("  "),
                Span::styled("InrComp: ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:>6.3}", telem.leader_inertia_comp)),
                Span::raw("  "),
                Span::styled("VelAst: ", Style::default().fg(Color::DarkGray)),
                Span::raw(format!("{:>6.3}", telem.leader_vel_assist)),
            ]));
        }
    } // end else (not ondemand)

    text.push(Line::from(vec![
        Span::raw(" "),
        Span::styled(
            match &telem.last_error {
                Some(e) => format!("ERR: {}", e),
                None => "OK".to_string(),
            },
            if telem.last_error.is_some() {
                Style::default().fg(Color::Red)
            } else {
                Style::default().fg(Color::Green)
            },
        ),
    ]));
    text.push(Line::from(Span::styled(
        " Press Esc or q to stop ",
        Style::default().fg(Color::DarkGray).add_modifier(Modifier::ITALIC),
    )));

    let area = frame.area();
    let overlay_height = (text.len() + 2) as u16; // +2 for borders
    let overlay_width = 70.min(area.width.saturating_sub(4));
    let overlay_area = Rect {
        x: (area.width.saturating_sub(overlay_width)) / 2,
        y: area.height.saturating_sub(overlay_height + 1),
        width: overlay_width,
        height: overlay_height,
    };

    let paragraph = Paragraph::new(text).block(
        Block::default()
            .borders(Borders::ALL)
            .title(overlay_title)
            .border_style(Style::default().fg(Color::Yellow)),
    );

    frame.render_widget(Clear, overlay_area);
    frame.render_widget(paragraph, overlay_area);
}
// =============================================================================
// Helpers
// =============================================================================

fn hex_str(data: &[u8]) -> String {
    data.iter().map(|b| format!("{:02X}", b)).collect::<Vec<_>>().join(" ")
}

fn chrono_like_timestamp() -> String {
    let dur = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default();
    let secs = dur.as_secs() % 86400;
    let h = secs / 3600;
    let m = (secs % 3600) / 60;
    let s = secs % 60;
    let ms = dur.subsec_millis();
    format!("{:02}:{:02}:{:02}.{:03}", h, m, s, ms)
}

fn parse_param_name(name: &str) -> Option<ParamIndex> {
    match name.to_lowercase().as_str() {
        "mech_pos" | "position" => Some(ParamIndex::MechPos),
        "mech_vel" | "velocity" => Some(ParamIndex::MechVel),
        "iq_filt" | "current" => Some(ParamIndex::IqFilt),
        "vbus" | "voltage" => Some(ParamIndex::Vbus),
        "limit_torque" => Some(ParamIndex::LimitTorque),
        "limit_spd" => Some(ParamIndex::LimitSpd),
        "limit_cur" => Some(ParamIndex::LimitCur),
        "run_mode" | "mode" => Some(ParamIndex::RunMode),
        "loc_kp" => Some(ParamIndex::LocKp),
        "spd_kp" => Some(ParamIndex::SpdKp),
        "spd_ki" => Some(ParamIndex::SpdKi),
        "loc_ref" => Some(ParamIndex::LocRef),
        "spd_ref" => Some(ParamIndex::SpdRef),
        "iq_ref" => Some(ParamIndex::IqRef),
        _ => None,
    }
}

// =============================================================================
// Main
// =============================================================================

fn main() -> Result<()> {
    // Parse simple CLI args for interface, host_id, model
    let args: Vec<String> = std::env::args().collect();
    let mut interface = "can0".to_string();
    let mut host_id: u8 = 0xFD;
    let mut model_str = "rs-05".to_string();

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "-i" | "--interface" => {
                if i + 1 < args.len() {
                    interface = args[i + 1].clone();
                    i += 1;
                }
            }
            "--host-id" => {
                if i + 1 < args.len() {
                    host_id = args[i + 1].parse().unwrap_or(0xFD);
                    i += 1;
                }
            }
            "--model" => {
                if i + 1 < args.len() {
                    model_str = args[i + 1].clone();
                    i += 1;
                }
            }
            "-h" | "--help" => {
                eprintln!("Usage: robstride_tui [-i can0] [--host-id 253] [--model rs-05]");
                eprintln!();
                eprintln!("TUI Controls:");
                eprintln!("  Tab        Switch panel (Motors / Commands)");
                eprintln!("  Up/Down    Navigate list");
                eprintln!("  Enter      Execute command / Read status");
                eprintln!("  Esc        Cancel input");
                eprintln!("  q / Ctrl-C Quit");
                std::process::exit(0);
            }
            _ => {}
        }
        i += 1;
    }

    let model = MotorModel::from_str(&model_str).unwrap_or(MotorModel::Rs05);

    // Setup terminal
    terminal::enable_raw_mode()?;
    let mut stdout = io::stdout();
    stdout.execute(EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut app = App::new(&interface, host_id, model);

    // Main loop
    let tick_rate = Duration::from_millis(50);
    let mut last_tick = Instant::now();

    loop {
        terminal.draw(|f| ui(f, &app))?;

        let timeout = tick_rate.saturating_sub(last_tick.elapsed());
        if crossterm::event::poll(timeout)? {
            if let Event::Key(key) = event::read()? {
                app.handle_key(key);
            }
        }

        if last_tick.elapsed() >= tick_rate {
            last_tick = Instant::now();
        }

        // Check for background scan completion
        app.check_scan_complete();

        // Periodic motor status refresh
        if app.last_refresh.elapsed() >= Duration::from_millis(app.refresh_interval_ms) {
            app.refresh_motor_status();
            app.last_refresh = Instant::now();
        }

        if app.quit {
            // Stop bilateral control if running
            if app.bilateral_active() {
                app.stop_bilateral();
            }
            // Disable all enabled motors before quitting (vendor-agnostic).
            if let Ok(socket) = CanSocket::open(&app.interface) {
                let _ = socket.set_read_timeout(Duration::from_millis(50));
                for entry in &app.motors {
                    if entry.enabled {
                        let mut driver = entry.spec.clone().build();
                        let _ = driver.disable(&socket);
                    }
                }
            }
            break;
        }
    }

    // Restore terminal
    terminal::disable_raw_mode()?;
    io::stdout().execute(LeaveAlternateScreen)?;

    Ok(())
}
