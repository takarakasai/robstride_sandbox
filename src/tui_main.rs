//! Robstride Motor Control TUI
//!
//! Interactive terminal application for controlling Robstride motors via CAN bus.
//! Uses Ratatui + Crossterm for the terminal interface.

use std::io;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::Result;
use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyModifiers};
use crossterm::terminal::{self, EnterAlternateScreen, LeaveAlternateScreen};
use crossterm::ExecutableCommand;
use ratatui::prelude::*;
use ratatui::widgets::*;

use robstride_sandbox::bilateral::{self, BilateralConfig, BilateralGains, BilateralMethod, SharedTelemetry, StopFlag};
use robstride_sandbox::motor::Motor;
use robstride_sandbox::protocol::{MotorFeedback, MotorModel, ParamIndex, RunMode};

// =============================================================================
// App state
// =============================================================================

/// Identifiable motor on the CAN bus.
#[derive(Debug, Clone)]
struct MotorEntry {
    id: u8,
    model: MotorModel,
    host_id: u8,
    enabled: bool,
    feedback: Option<MotorFeedback>,
    last_update: Option<Instant>,
    uuid: Option<Vec<u8>>,
    error: Option<String>,
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
            desc,
            choices: None,
        }
    }

    fn with_choices(name: &'static str, value: &str, desc: &'static str, choices: &'static str) -> Self {
        ParamField {
            name,
            value: value.to_string(),
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
            ParamField::with_choices("method", "coupling", "Control method",
                "pos|force|coupling|mode"),
            ParamField::new("kp", "5.0", "Spring stiffness [Nm/rad]"),
            ParamField::new("kd", "0.3", "Damping [Nm·s/rad]"),
            ParamField::new("coulomb", "0.05", "Coulomb friction comp [Nm]"),
            ParamField::new("viscous", "0.01", "Viscous friction comp [Nm·s/rad]"),
            ParamField::new("force_sc", "0.5", "Force reflection scale (force method)"),
            ParamField::new("inertia", "0.005", "Motor inertia [kg·m²] (mode method)"),
            ParamField::new("dob_cut", "100.0", "DOB cutoff [rad/s] (mode method)"),
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
}

impl Command {
    const ALL: [Command; 14] = [
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
}

impl App {
    fn new(interface: &str, host_id: u8, model: MotorModel) -> Self {
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
            params: params_for_command(Command::ALL[0]),
            selected_param: 0,
            editing_param: false,
            param_edit_buf: String::new(),
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
        self.selected_motor_entry().map(|m| m.id)
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

        // Parse range from input
        let parts: Vec<&str> = input.trim().split_whitespace().collect();
        let from: u8 = if !parts.is_empty() {
            parts[0].parse().unwrap_or(1)
        } else {
            1
        };
        let to: u8 = if parts.len() > 1 {
            parts[1].parse().unwrap_or(127)
        } else {
            127
        };
        let from = from.max(1);
        let to = to.max(from).min(254);

        self.log_msg(format!("Scanning CAN bus ID {}..={}...", from, to));

        // Reset progress
        {
            let mut progress = self.scan_progress.lock().unwrap();
            *progress = (0, (to - from + 1) as usize, true);
        }
        {
            let mut results = self.scan_results.lock().unwrap();
            results.clear();
        }

        // Launch scan in background thread
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
                p.2 = false; // scanning done
            }
        });
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
                let exists = self.motors.iter().any(|m| m.id == id);
                if !exists {
                    let entry = MotorEntry {
                        id,
                        model: self.default_model,
                        host_id: self.host_id,
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
            self.motors.sort_by_key(|m| m.id);
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
        let Some(mid) = self.selected_motor_id() else {
            self.log_msg("No motor selected. Run Scan first.".to_string());
            return;
        };
        let entry = &self.motors[self.selected_motor];
        match Motor::new(&self.interface, mid, entry.host_id, entry.model) {
            Ok(motor) => match motor.ping() {
                Ok((device_id, uuid)) => {
                    self.log_msg(format!(
                        "Ping OK: motor={} device_id=0x{:04X} UUID=[{}]",
                        mid, device_id, hex_str(&uuid)
                    ));
                    self.motors[self.selected_motor].uuid = Some(uuid);
                }
                Err(e) => self.log_msg(format!("Ping failed: {}", e)),
            },
            Err(e) => self.log_msg(format!("CAN open error: {}", e)),
        }
    }

    fn execute_enable(&mut self) {
        let Some(mid) = self.selected_motor_id() else {
            self.log_msg("No motor selected.".to_string());
            return;
        };
        let entry = &self.motors[self.selected_motor];
        match Motor::new(&self.interface, mid, entry.host_id, entry.model) {
            Ok(mut motor) => match motor.enable() {
                Ok(fb) => {
                    self.log_msg(format!("Motor {} enabled.", mid));
                    self.motors[self.selected_motor].enabled = true;
                    self.motors[self.selected_motor].feedback = Some(fb);
                    self.motors[self.selected_motor].last_update = Some(Instant::now());
                    // Prevent double-disable in Motor::drop
                    std::mem::forget(motor);
                }
                Err(e) => self.log_msg(format!("Enable failed: {}", e)),
            },
            Err(e) => self.log_msg(format!("CAN open error: {}", e)),
        }
    }

    fn execute_disable(&mut self) {
        let Some(mid) = self.selected_motor_id() else {
            self.log_msg("No motor selected.".to_string());
            return;
        };
        let entry = &self.motors[self.selected_motor];
        match Motor::new(&self.interface, mid, entry.host_id, entry.model) {
            Ok(mut motor) => {
                // Tell Motor it's enabled so disable() works
                match motor.disable() {
                    Ok(fb) => {
                        self.log_msg(format!("Motor {} disabled.", mid));
                        self.motors[self.selected_motor].enabled = false;
                        self.motors[self.selected_motor].feedback = Some(fb);
                        self.motors[self.selected_motor].last_update = Some(Instant::now());
                    }
                    Err(e) => self.log_msg(format!("Disable failed: {}", e)),
                }
                std::mem::forget(motor);
            }
            Err(e) => self.log_msg(format!("CAN open error: {}", e)),
        }
    }

    fn execute_set_zero(&mut self) {
        let Some(mid) = self.selected_motor_id() else {
            self.log_msg("No motor selected.".to_string());
            return;
        };
        let entry = &self.motors[self.selected_motor];
        match Motor::new(&self.interface, mid, entry.host_id, entry.model) {
            Ok(mut motor) => match motor.set_zero() {
                Ok(()) => {
                    self.log_msg(format!("Motor {} zero set.", mid));
                }
                Err(e) => self.log_msg(format!("Set zero failed: {}", e)),
            },
            Err(e) => self.log_msg(format!("CAN open error: {}", e)),
        }
    }

    fn execute_read_status(&mut self) {
        let Some(mid) = self.selected_motor_id() else {
            self.log_msg("No motor selected.".to_string());
            return;
        };
        let entry = &self.motors[self.selected_motor];
        match Motor::new(&self.interface, mid, entry.host_id, entry.model) {
            Ok(motor) => match motor.read_status() {
                Ok(fb) => {
                    self.log_msg(format!(
                        "Status: pos={:.4} vel={:.4} torque={:.4} temp={:.1}°C mode={}",
                        fb.position, fb.velocity, fb.torque, fb.temperature, fb.status.mode
                    ));
                    self.motors[self.selected_motor].feedback = Some(fb);
                    self.motors[self.selected_motor].last_update = Some(Instant::now());
                    self.motors[self.selected_motor].error = None;
                }
                Err(e) => {
                    self.log_msg(format!("Read status failed: {}", e));
                    self.motors[self.selected_motor].error = Some(e.to_string());
                }
            },
            Err(e) => self.log_msg(format!("CAN open error: {}", e)),
        }
    }

    fn execute_read_param(&mut self, input: &str) {
        let Some(mid) = self.selected_motor_id() else {
            self.log_msg("No motor selected.".to_string());
            return;
        };
        let param = match parse_param_name(input.trim()) {
            Some(p) => p,
            None => {
                self.log_msg(format!("Unknown param: '{}'. Available: mech_pos, mech_vel, iq_filt, vbus, limit_torque, limit_spd, limit_cur, run_mode, loc_kp, spd_kp, spd_ki", input));
                return;
            }
        };
        let entry = &self.motors[self.selected_motor];
        match Motor::new(&self.interface, mid, entry.host_id, entry.model) {
            Ok(motor) => match motor.read_param(param) {
                Ok(val) => self.log_msg(format!("{} = {:.4}", input.trim(), val)),
                Err(e) => self.log_msg(format!("Read param failed: {}", e)),
            },
            Err(e) => self.log_msg(format!("CAN open error: {}", e)),
        }
    }

    fn execute_write_param(&mut self, input: &str) {
        let Some(mid) = self.selected_motor_id() else {
            self.log_msg("No motor selected.".to_string());
            return;
        };
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
        let entry = &self.motors[self.selected_motor];
        match Motor::new(&self.interface, mid, entry.host_id, entry.model) {
            Ok(motor) => match motor.write_param_f32(param, value) {
                Ok(()) => self.log_msg(format!("{} = {:.4} (written)", parts[0], value)),
                Err(e) => self.log_msg(format!("Write param failed: {}", e)),
            },
            Err(e) => self.log_msg(format!("CAN open error: {}", e)),
        }
    }

    fn execute_set_run_mode(&mut self, input: &str) {
        let Some(mid) = self.selected_motor_id() else {
            self.log_msg("No motor selected.".to_string());
            return;
        };
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
        let entry = &self.motors[self.selected_motor];
        match Motor::new(&self.interface, mid, entry.host_id, entry.model) {
            Ok(mut motor) => match motor.set_run_mode(mode) {
                Ok(()) => self.log_msg(format!("Run mode set to {:?}", mode)),
                Err(e) => self.log_msg(format!("Set run mode failed: {}", e)),
            },
            Err(e) => self.log_msg(format!("CAN open error: {}", e)),
        }
    }

    fn execute_move_to(&mut self, input: &str) {
        let Some(mid) = self.selected_motor_id() else {
            self.log_msg("No motor selected.".to_string());
            return;
        };
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

        let entry = &self.motors[self.selected_motor];
        match Motor::new(&self.interface, mid, entry.host_id, entry.model) {
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
                        self.motors[self.selected_motor].enabled = true;
                    }
                    Err(e) => self.log_msg(format!("Move to failed: {}", e)),
                }
                std::mem::forget(motor);
            }
            Err(e) => self.log_msg(format!("CAN open error: {}", e)),
        }
    }

    fn execute_spin(&mut self, input: &str) {
        let Some(mid) = self.selected_motor_id() else {
            self.log_msg("No motor selected.".to_string());
            return;
        };
        let vel: f32 = match input.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.log_msg("Invalid velocity value.".to_string());
                return;
            }
        };
        let entry = &self.motors[self.selected_motor];
        match Motor::new(&self.interface, mid, entry.host_id, entry.model) {
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
                        self.motors[self.selected_motor].enabled = true;
                    }
                    Err(e) => self.log_msg(format!("Spin failed: {}", e)),
                }
                std::mem::forget(motor);
            }
            Err(e) => self.log_msg(format!("CAN open error: {}", e)),
        }
    }

    fn execute_torque(&mut self, input: &str) {
        let Some(mid) = self.selected_motor_id() else {
            self.log_msg("No motor selected.".to_string());
            return;
        };
        let torque: f32 = match input.trim().parse() {
            Ok(v) => v,
            Err(_) => {
                self.log_msg("Invalid torque value.".to_string());
                return;
            }
        };
        let entry = &self.motors[self.selected_motor];
        match Motor::new(&self.interface, mid, entry.host_id, entry.model) {
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
                        self.motors[self.selected_motor].enabled = true;
                    }
                    Err(e) => self.log_msg(format!("Torque failed: {}", e)),
                }
                std::mem::forget(motor);
            }
            Err(e) => self.log_msg(format!("CAN open error: {}", e)),
        }
    }

    fn execute_mit(&mut self, input: &str) {
        let Some(mid) = self.selected_motor_id() else {
            self.log_msg("No motor selected.".to_string());
            return;
        };
        let parts: Vec<f64> = input
            .trim()
            .split_whitespace()
            .filter_map(|s| s.parse().ok())
            .collect();
        if parts.len() != 5 {
            self.log_msg("Usage: pos vel kp kd torque (5 values)".to_string());
            return;
        }
        let entry = &self.motors[self.selected_motor];
        match Motor::new(&self.interface, mid, entry.host_id, entry.model) {
            Ok(mut motor) => {
                let r = (|| -> std::result::Result<MotorFeedback, robstride_sandbox::error::RobstrideError> {
                    if !self.motors[self.selected_motor].enabled {
                        motor.enable()?;
                        self.motors[self.selected_motor].enabled = true;
                    }
                    motor.mit_control(parts[0], parts[1], parts[2], parts[3], parts[4])
                })();
                match r {
                    Ok(fb) => {
                        self.log_msg(format!(
                            "MIT: pos={:.4} vel={:.4} torque={:.4}",
                            fb.position, fb.velocity, fb.torque
                        ));
                        self.motors[self.selected_motor].feedback = Some(fb);
                        self.motors[self.selected_motor].last_update = Some(Instant::now());
                    }
                    Err(e) => self.log_msg(format!("MIT control failed: {}", e)),
                }
                std::mem::forget(motor);
            }
            Err(e) => self.log_msg(format!("CAN open error: {}", e)),
        }
    }

    fn execute_bilateral(&mut self, input: &str) {
        // If already running, stop it
        if self.bilateral_stop.is_some() {
            self.stop_bilateral();
            return;
        }

        // Parse: method [kp kd [coulomb_friction viscous_friction] [force_scale] [inertia dob_cutoff]]
        let parts: Vec<&str> = input.trim().split_whitespace().collect();
        let method_str = if !parts.is_empty() { parts[0] } else { "coupling" };
        let method = match BilateralMethod::from_short(method_str) {
            Some(m) => m,
            None => {
                self.log_msg(format!(
                    "Unknown method '{}'. Use: pos, force, coupling, mode",
                    method_str
                ));
                return;
            }
        };

        let mut gains = BilateralGains::default();
        if parts.len() > 1 {
            gains.kp = parts[1].parse().unwrap_or(gains.kp);
        }
        if parts.len() > 2 {
            gains.kd = parts[2].parse().unwrap_or(gains.kd);
        }
        if parts.len() > 3 {
            gains.coulomb_friction = parts[3].parse().unwrap_or(gains.coulomb_friction);
        }
        if parts.len() > 4 {
            gains.viscous_friction = parts[4].parse().unwrap_or(gains.viscous_friction);
        }
        if parts.len() > 5 {
            gains.force_scale = parts[5].parse().unwrap_or(gains.force_scale);
        }
        if parts.len() > 6 {
            gains.inertia = parts[6].parse().unwrap_or(gains.inertia);
        }
        if parts.len() > 7 {
            gains.dob_cutoff = parts[7].parse().unwrap_or(gains.dob_cutoff);
        }

        let config = BilateralConfig {
            interface: self.interface.clone(),
            host_id: self.host_id,
            leader_id: 10,
            follower_id: 1,
            model: self.default_model,
            method,
            gains,
            loop_period_us: 2000,
        };

        self.log_msg(format!(
            "Starting bilateral control: {} (Kp={:.2}, Kd={:.2}, Cf={:.3}, Vf={:.3})",
            method.label(),
            gains.kp,
            gains.kd,
            gains.coulomb_friction,
            gains.viscous_friction,
        ));
        self.log_msg(format!(
            "  Leader=ID:{}, Follower=ID:{}  Press Esc to stop.",
            config.leader_id,
            config.follower_id,
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
        }
    }

    /// Refresh status for all enabled motors.
    fn refresh_motor_status(&mut self) {
        for i in 0..self.motors.len() {
            let entry = &self.motors[i];
            if !entry.enabled {
                continue;
            }
            let mid = entry.id;
            let host_id = entry.host_id;
            let model = entry.model;
            match Motor::new(&self.interface, mid, host_id, model) {
                Ok(motor) => match motor.read_status() {
                    Ok(fb) => {
                        self.motors[i].feedback = Some(fb);
                        self.motors[i].last_update = Some(Instant::now());
                        self.motors[i].error = None;
                    }
                    Err(e) => {
                        self.motors[i].error = Some(e.to_string());
                    }
                },
                Err(_) => {}
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
        self.params = merged;
        self.selected_param = 0;
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
    let cmd = Command::ALL[app.selected_cmd];
    let top_chunks = if cmd.has_params() {
        Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(45),
                Constraint::Percentage(18),
                Constraint::Percentage(37),
            ])
            .split(main_chunks[0])
    } else {
        Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(65),
                Constraint::Percentage(35),
                Constraint::Length(0),
            ])
            .split(main_chunks[0])
    };

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
    if cmd.has_params() {
        render_params(frame, app, top_chunks[2]);
    }
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
                    Cell::from(format!("{}", m.id)),
                    Cell::from(format!("{}", m.model)),
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

        // Choice indicator
        let choice_marker = if param.choices.is_some() { "▼" } else { "" };

        let line = Line::from(vec![
            Span::styled(
                format!(" {:>width$} ", param.name, width = name_width),
                name_style,
            ),
            Span::styled(val_str, val_style),
            Span::styled(
                format!(" {}", choice_marker),
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

    let method_name = telem
        .method
        .map(|m| m.label())
        .unwrap_or("???");

    let text = vec![
        Line::from(vec![
            Span::styled(" Method: ", Style::default().fg(Color::Yellow)),
            Span::styled(method_name, Style::default().fg(Color::White).add_modifier(Modifier::BOLD)),
            Span::raw("    "),
            Span::styled(format!("Loop: {:.0} Hz", telem.loop_hz), Style::default().fg(Color::Cyan)),
            Span::raw("    "),
            Span::styled(format!("Cycles: {}", telem.cycle_count), Style::default().fg(Color::DarkGray)),
        ]),
        Line::from(vec![
            Span::styled(" Leader  (10): ", Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
            Span::raw(format!(
                "pos={:>8.3}  vel={:>8.3}  τcmd={:>7.3}",
                telem.leader_pos, telem.leader_vel, telem.leader_torque_cmd,
            )),
        ]),
        Line::from(vec![
            Span::styled(" Follow  ( 1): ", Style::default().fg(Color::Magenta).add_modifier(Modifier::BOLD)),
            Span::raw(format!(
                "pos={:>8.3}  vel={:>8.3}  τcmd={:>7.3}",
                telem.follower_pos, telem.follower_vel, telem.follower_torque_cmd,
            )),
        ]),
        Line::from(vec![
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
        ]),
        Line::from(vec![
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
        ]),
        Line::from(Span::styled(
            " Press Esc or q to stop bilateral control ",
            Style::default().fg(Color::DarkGray).add_modifier(Modifier::ITALIC),
        )),
    ];

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
            .title(" Bilateral Control ")
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
            // Disable all enabled motors before quitting
            for entry in &app.motors {
                if entry.enabled {
                    if let Ok(mut motor) =
                        Motor::new(&app.interface, entry.id, entry.host_id, entry.model)
                    {
                        let _ = motor.disable();
                        std::mem::forget(motor);
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
