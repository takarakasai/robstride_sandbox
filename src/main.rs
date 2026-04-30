//! Motor control CLI for Robstride and DAMIAO CAN motors.
//!
//! Usage:
//!   robstride_sandbox --interface can0 --motor-id 1 --model rs-05 <COMMAND>

use anyhow::{bail, Result};
use clap::{Parser, Subcommand, ValueEnum};
use robstride_sandbox::driver::{self, DamiaoModel, MotorSpec};
use robstride_sandbox::motor::Motor;
use robstride_sandbox::protocol::{
    parse_can_id, MotorFeedback, MotorModel, ParamIndex, RunMode, DEFAULT_HOST_ID,
};
use robstride_sandbox::serial_can::SerialCan;
use robstride_sandbox::transport::CanBus;
use std::time::{Duration, Instant};

const DEFAULT_DM_MODEL: DamiaoModel = DamiaoModel::DmJ4310_2EC;
const DM_MOVE_KP: f64 = 20.0;
const DM_MOVE_KD: f64 = 1.0;
const DM_SPIN_KD: f64 = 1.0;
const DM_CONTROL_PERIOD: Duration = Duration::from_millis(20);

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum Vendor {
    Auto,
    Robstride,
    Damiao,
    Both,
}

#[derive(Debug, Clone, Copy)]
enum SelectedMotor {
    Robstride(MotorModel),
    Damiao(DamiaoModel),
}

impl SelectedMotor {
    fn description(self) -> String {
        match self {
            SelectedMotor::Robstride(model) => format!("Robstride {}", model),
            SelectedMotor::Damiao(model) => format!("DAMIAO {}", model),
        }
    }
}

#[derive(Parser)]
#[command(name = "robstride_sandbox")]
#[command(about = "Robstride / DAMIAO motor controller CLI")]
struct Cli {
    /// CAN interface name (e.g., can0)
    #[arg(short, long, default_value = "can0")]
    interface: String,

    /// Motor CAN ID (1-254)
    #[arg(short, long, default_value_t = 1)]
    motor_id: u8,

    /// Host CAN ID for Robstride commands
    #[arg(long, default_value_t = DEFAULT_HOST_ID)]
    host_id: u8,

    /// Use CAN FD framing (BRS on) for all bus traffic instead of Classic CAN
    #[arg(long, default_value_t = false)]
    fd: bool,

    /// Motor vendor (`auto` infers from `--model`, `both` is scan-only)
    #[arg(long, value_enum, default_value_t = Vendor::Auto)]
    vendor: Vendor,

    /// Motor model (Robstride: rs-00..rs-06, DAMIAO: dm-j4310-2ec)
    #[arg(long, default_value = "rs-05")]
    model: String,

    /// DAMIAO response standard ID (MST_ID). Use 0 to accept any responder.
    #[arg(long, default_value_t = 0)]
    master_id: u16,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Ping the motor
    Ping,

    /// Read motor status
    Status,

    /// Enable the motor
    Enable,

    /// Disable the motor
    Disable,

    /// Set current position as zero
    SetZero,

    /// Move to a position
    MoveTo {
        /// Target position in radians
        position: f32,
        /// Speed limit / velocity clamp in rad/s
        #[arg(short, long, default_value_t = 5.0)]
        speed: f32,
    },

    /// Set velocity
    Spin {
        /// Target velocity in rad/s
        velocity: f32,
        /// Duration in seconds (0 = indefinite)
        #[arg(short, long, default_value_t = 0.0)]
        duration: f32,
    },

    /// Apply torque
    Torque {
        /// Target torque in Nm
        torque: f32,
        /// Duration in seconds (0 = indefinite)
        #[arg(short, long, default_value_t = 0.0)]
        duration: f32,
    },

    /// MIT-mode control
    Mit {
        /// Target position [rad]
        #[arg(long, default_value_t = 0.0)]
        pos: f64,
        /// Target velocity [rad/s]
        #[arg(long, default_value_t = 0.0)]
        vel: f64,
        /// Position gain Kp
        #[arg(long, default_value_t = 0.0)]
        kp: f64,
        /// Velocity gain Kd
        #[arg(long, default_value_t = 0.0)]
        kd: f64,
        /// Feed-forward torque [Nm]
        #[arg(long, default_value_t = 0.0)]
        torque: f64,
    },

    /// Read a parameter
    ReadParam {
        /// Parameter name
        name: String,
    },

    /// Write a parameter
    WriteParam {
        /// Parameter name
        name: String,
        /// Value to write
        value: f32,
    },

    /// Continuous status monitoring
    Monitor {
        /// Update interval in milliseconds
        #[arg(short, long, default_value_t = 100)]
        interval: u64,
    },

    /// Scan the CAN bus for connected motors
    Scan {
        /// Start of motor ID range to scan
        #[arg(long, default_value_t = 1)]
        from: u8,
        /// End of motor ID range to scan (inclusive)
        #[arg(long, default_value_t = 127)]
        to: u8,
        /// Timeout per motor ID in milliseconds
        #[arg(short, long, default_value_t = 100)]
        timeout: u64,
    },

    /// Dump raw CAN bus traffic (passive listen)
    Dump {
        /// Duration to listen in seconds
        #[arg(short, long, default_value_t = 5.0)]
        duration: f32,
    },

    /// Scan motors via direct serial port (Robstride only)
    SerialScan {
        /// Serial device path
        #[arg(long, default_value = "/dev/ttyUSB0")]
        device: String,
        /// Serial baud rate
        #[arg(long, default_value_t = 921600)]
        baud: u32,
        /// CAN bus bitrate
        #[arg(long, default_value_t = 1_000_000)]
        can_bitrate: u32,
        /// Start of motor ID range
        #[arg(long, default_value_t = 1)]
        from: u8,
        /// End of motor ID range (inclusive)
        #[arg(long, default_value_t = 127)]
        to: u8,
        /// Timeout per motor ID in milliseconds
        #[arg(short, long, default_value_t = 100)]
        timeout: u64,
    },

    /// Dump raw serial port bytes for adapter protocol identification
    SerialDump {
        /// Serial device path
        #[arg(long, default_value = "/dev/ttyUSB0")]
        device: String,
        /// Serial baud rate to try
        #[arg(long, default_value_t = 921600)]
        baud: u32,
        /// Duration in seconds
        #[arg(short, long, default_value_t = 5.0)]
        duration: f32,
    },
}

fn main() -> Result<()> {
    env_logger::init();
    let cli = Cli::parse();

    match &cli.command {
        Commands::Scan { from, to, timeout } => {
            run_scan(&cli, *from, *to, *timeout)?;
            return Ok(());
        }
        Commands::Dump { duration } => {
            run_dump(&cli.interface, *duration, cli.fd)?;
            return Ok(());
        }
        Commands::SerialScan {
            device,
            baud,
            can_bitrate,
            from,
            to,
            timeout,
        } => return run_serial_scan(&cli, device, *baud, *can_bitrate, *from, *to, *timeout),
        Commands::SerialDump {
            device,
            baud,
            duration,
        } => {
            if matches!(cli.vendor, Vendor::Damiao | Vendor::Both) {
                bail!("serial-dump currently supports Robstride adapters only");
            }
            SerialCan::dump_raw_pretty(device, *baud, Duration::from_secs_f32(*duration));
            return Ok(());
        }
        _ => {}
    }

    let target = resolve_motor(&cli)?;
    println!(
        "Connected to CAN '{}', motor ID={}, host=0x{:02X}, target={}",
        cli.interface,
        cli.motor_id,
        cli.host_id,
        target.description()
    );

    match target {
        SelectedMotor::Robstride(model) => run_robstride_command(&cli, model, &cli.command),
        SelectedMotor::Damiao(model) => run_damiao_command(&cli, model, &cli.command),
    }
}

fn resolve_motor(cli: &Cli) -> Result<SelectedMotor> {
    match cli.vendor {
        Vendor::Both => bail!("--vendor both is only valid with the scan command"),
        Vendor::Robstride => {
            let model = MotorModel::from_str(&cli.model).ok_or_else(|| {
                anyhow::anyhow!("unknown Robstride model '{}'; use rs-00..rs-06", cli.model)
            })?;
            Ok(SelectedMotor::Robstride(model))
        }
        Vendor::Damiao => {
            let model = DamiaoModel::from_str_ci(&cli.model).unwrap_or(DEFAULT_DM_MODEL);
            Ok(SelectedMotor::Damiao(model))
        }
        Vendor::Auto => {
            if let Some(model) = MotorModel::from_str(&cli.model) {
                Ok(SelectedMotor::Robstride(model))
            } else if let Some(model) = DamiaoModel::from_str_ci(&cli.model) {
                Ok(SelectedMotor::Damiao(model))
            } else {
                bail!(
                    "unknown model '{}'; Robstride: rs-00..rs-06, DAMIAO: dm-j4310-2ec",
                    cli.model
                )
            }
        }
    }
}

fn run_scan(cli: &Cli, from: u8, to: u8, timeout_ms: u64) -> Result<()> {
    let timeout = Duration::from_millis(timeout_ms);
    let mut found_any = false;

    if matches!(cli.vendor, Vendor::Auto | Vendor::Robstride | Vendor::Both) {
        println!(
            "Scanning Robstride motors on '{}' (ID {}..={}, timeout {}ms/id, host=0x{:02X})...",
            cli.interface, from, to, timeout_ms, cli.host_id
        );
        let results = Motor::scan_bus(&cli.interface, cli.host_id, from..=to, timeout, cli.fd);
        if results.is_empty() {
            println!("  No Robstride motors found.");
        } else {
            found_any = true;
            println!("  Found {} Robstride motor(s):", results.len());
            for (id, data) in &results {
                match data {
                    Some(bytes) => {
                        let hex: Vec<String> = bytes.iter().map(|b| format!("{:02X}", b)).collect();
                        println!("    ID={}: data=[{}]", id, hex.join(" "));
                    }
                    None => println!("    ID={}: (responded, no data)", id),
                }
            }
        }
    }

    if matches!(cli.vendor, Vendor::Auto | Vendor::Damiao | Vendor::Both) {
        println!(
            "Scanning DAMIAO motors on '{}' (ID {}..={}, timeout {}ms/id)...",
            cli.interface, from, to, timeout_ms
        );
        let ids = driver::scan_damiao(&cli.interface, from..=to, timeout, cli.fd)?;
        if ids.is_empty() {
            println!("  No DAMIAO motors found.");
        } else {
            found_any = true;
            println!("  Found {} DAMIAO motor(s):", ids.len());
            for id in ids {
                println!("    ID={}", id);
            }
        }
    }

    if !found_any {
        println!("\nNo motors found.");
        println!("\nTroubleshooting:");
        println!("  1. Is the CAN interface up?  ip link show {}", cli.interface);
        println!("  2. Is the bitrate correct?   (Robstride default: 1Mbps)");
        println!("  3. Is the motor powered on?");
        println!("  4. Check wiring (CAN-H, CAN-L, GND)");
        println!("  5. Try passive listen: cargo run -- -i {} dump", cli.interface);
    }

    Ok(())
}

fn run_dump(interface: &str, duration_secs: f32, fd: bool) -> Result<()> {
    println!(
        "Listening on '{}' for {:.1}s (passive, no frames sent)...",
        interface, duration_secs
    );

    let socket = CanBus::open(interface, fd)?;
    socket.set_read_timeout(Duration::from_millis(100))?;
    let deadline = Instant::now() + Duration::from_secs_f32(duration_secs);
    let mut frames = Vec::new();

    while Instant::now() < deadline {
        match socket.read_frame() {
            Ok(frame) => {
                frames.push((frame.extended, frame.raw_id, frame.data));
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
            Err(e) => return Err(e.into()),
        }
    }

    if frames.is_empty() {
        println!("\nNo CAN frames received.");
        return Ok(());
    }

    println!("\nReceived {} frame(s):", frames.len());
    println!("{:>5} {:>12}  {:>6} {:>8} {:>4}  Payload", "Fmt", "CAN ID", "Type", "Extra", "Dev");
    println!("{}", "-".repeat(72));

    for (is_extended, can_id, data) in frames {
        let hex_data: Vec<String> = data.iter().map(|b| format!("{:02X}", b)).collect();
        if is_extended {
            let (ct, extra, dev) = parse_can_id(can_id);
            println!(
                "{:>5}  0x{:08X}  {:>6}  0x{:04X} {:>4}  [{}]",
                "EXT",
                can_id,
                ct,
                extra,
                dev,
                hex_data.join(" ")
            );
        } else {
            println!(
                "{:>5}  0x{:08X}  {:>6} {:>8} {:>4}  [{}]",
                "STD",
                can_id,
                "-",
                "-",
                "-",
                hex_data.join(" ")
            );
        }
    }

    Ok(())
}

fn run_serial_scan(
    cli: &Cli,
    device: &str,
    baud: u32,
    can_bitrate: u32,
    from: u8,
    to: u8,
    timeout_ms: u64,
) -> Result<()> {
    if matches!(cli.vendor, Vendor::Damiao | Vendor::Both) {
        bail!("serial-scan currently supports Robstride adapters only");
    }

    println!(
        "Serial scan: {} @ {}baud, CAN {}bps, ID {}..={}",
        device, baud, can_bitrate, from, to
    );
    let mut sc = SerialCan::open(device, baud, can_bitrate)
        .map_err(|e| anyhow::anyhow!("Failed to open serial CAN: {}", e))?;
    let results = sc.scan(cli.host_id, from..=to, Duration::from_millis(timeout_ms));

    if results.is_empty() {
        println!("\nNo motors found via serial.");
    } else {
        println!("\nFound {} motor(s):", results.len());
        for (id, feedback) in &results {
            match feedback {
                Some(fb) => println!(
                    "  ID={}: pos={:.3} vel={:.3} torque={:.3} temp={:.1}",
                    id, fb.position, fb.velocity, fb.torque, fb.temperature
                ),
                None => println!("  ID={}: (responded but no feedback data)", id),
            }
        }
    }

    Ok(())
}

fn run_robstride_command(cli: &Cli, model: MotorModel, command: &Commands) -> Result<()> {
    let mut motor = Motor::new(&cli.interface, cli.motor_id, cli.host_id, model, cli.fd)?;

    match command {
        Commands::Ping => match motor.ping() {
            Ok((device_id, uuid)) => {
                let hex: Vec<String> = uuid.iter().map(|b| format!("{:02X}", b)).collect();
                println!(
                    "Motor responded! device_id=0x{:04X} UUID=[{}]",
                    device_id,
                    hex.join(" ")
                );
            }
            Err(e) => println!("No response from motor: {}", e),
        },
        Commands::Status => {
            let fb = motor.read_status()?;
            print_feedback(&fb);
        }
        Commands::Enable => {
            let fb = motor.enable()?;
            println!("Motor enabled.");
            print_feedback(&fb);
        }
        Commands::Disable => {
            let fb = motor.disable()?;
            println!("Motor disabled.");
            print_feedback(&fb);
        }
        Commands::SetZero => {
            motor.set_zero()?;
            println!("Mechanical zero set at current position.");
        }
        Commands::MoveTo { position, speed } => {
            motor.disable()?;
            motor.set_run_mode(RunMode::Position)?;
            motor.set_position_speed_limit(*speed)?;
            motor.enable()?;
            motor.set_position(*position)?;
            println!(
                "Moving to {:.3} rad (speed limit: {:.1} rad/s)",
                position, speed
            );

            loop {
                std::thread::sleep(Duration::from_millis(100));
                let fb = motor.read_status()?;
                print!(
                    "\r  pos={:.3} vel={:.3} torque={:.3}   ",
                    fb.position, fb.velocity, fb.torque
                );
                if (fb.position - *position as f64).abs() < 0.05 && fb.velocity.abs() < 0.1 {
                    println!("\nTarget reached.");
                    break;
                }
            }
        }
        Commands::Spin { velocity, duration } => {
            motor.disable()?;
            motor.set_run_mode(RunMode::Velocity)?;
            motor.enable()?;
            motor.set_velocity(*velocity)?;
            println!("Spinning at {:.2} rad/s", velocity);

            if *duration > 0.0 {
                std::thread::sleep(Duration::from_secs_f32(*duration));
                motor.set_velocity(0.0)?;
                std::thread::sleep(Duration::from_millis(500));
                motor.disable()?;
                println!("Stopped.");
            } else {
                println!("Press Ctrl+C to stop.");
                loop {
                    std::thread::sleep(Duration::from_millis(100));
                    let fb = motor.read_status()?;
                    print!(
                        "\r  pos={:.3} vel={:.3} torque={:.3}   ",
                        fb.position, fb.velocity, fb.torque
                    );
                }
            }
        }
        Commands::Torque { torque, duration } => {
            motor.disable()?;
            motor.set_run_mode(RunMode::Torque)?;
            motor.enable()?;
            motor.set_torque(*torque)?;
            println!("Applying torque: {:.3} Nm", torque);

            if *duration > 0.0 {
                std::thread::sleep(Duration::from_secs_f32(*duration));
                motor.set_torque(0.0)?;
                motor.disable()?;
                println!("Stopped.");
            } else {
                println!("Press Ctrl+C to stop.");
                loop {
                    std::thread::sleep(Duration::from_millis(100));
                    let fb = motor.read_status()?;
                    print!(
                        "\r  pos={:.3} vel={:.3} torque={:.3}   ",
                        fb.position, fb.velocity, fb.torque
                    );
                }
            }
        }
        Commands::Mit {
            pos,
            vel,
            kp,
            kd,
            torque,
        } => {
            if !motor.is_enabled() {
                motor.enable()?;
            }
            let fb = motor.mit_control(*pos, *vel, *kp, *kd, *torque)?;
            print_feedback(&fb);
        }
        Commands::ReadParam { name } => {
            let param = parse_read_param_name(name)?;
            let value = motor.read_param(param)?;
            println!("{} = {:.4}", name, value);
        }
        Commands::WriteParam { name, value } => {
            let param = parse_write_param_name(name)?;
            motor.write_param_f32(param, *value)?;
            println!("{} = {:.4} (written)", name, value);
        }
        Commands::Monitor { interval } => {
            println!("Monitoring motor status (interval: {}ms)...", interval);
            println!(
                "{:>10} {:>10} {:>10} {:>8}",
                "Pos[rad]", "Vel[r/s]", "Torq[Nm]", "Temp[°C]"
            );
            println!("{}", "-".repeat(45));

            loop {
                let fb = motor.read_status()?;
                print!(
                    "\r{:>10.3} {:>10.3} {:>10.3} {:>8.1}",
                    fb.position, fb.velocity, fb.torque, fb.temperature
                );
                std::thread::sleep(Duration::from_millis(*interval));
            }
        }
        Commands::Scan { .. }
        | Commands::Dump { .. }
        | Commands::SerialScan { .. }
        | Commands::SerialDump { .. } => unreachable!(),
    }

    Ok(())
}

fn run_damiao_command(cli: &Cli, model: DamiaoModel, command: &Commands) -> Result<()> {
    let spec = MotorSpec::damiao(cli.motor_id, cli.master_id, model);

    match command {
        Commands::Ping => {
            let fb = damiao_feedback_once(&cli.interface, &spec, cli.fd)?;
            println!("Motor responded.");
            print_feedback(&fb);
        }
        Commands::Status => {
            let fb = damiao_feedback_once(&cli.interface, &spec, cli.fd)?;
            print_feedback(&fb);
        }
        Commands::Enable => {
            let socket = open_can_socket(&cli.interface, cli.fd)?;
            let mut driver = spec.build();
            driver.enable(&socket)?;
            println!("Motor enabled.");
            let fb = driver.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0)?;
            print_feedback(&fb);
        }
        Commands::Disable => {
            let socket = open_can_socket(&cli.interface, cli.fd)?;
            let mut driver = spec.build();
            driver.disable(&socket)?;
            println!("Motor disabled.");
        }
        Commands::SetZero => {
            let socket = open_can_socket(&cli.interface, cli.fd)?;
            driver::damiao_set_zero_nvm(&socket, cli.motor_id)?;
            println!("DAMIAO hardware zero saved to NVM.");
        }
        Commands::MoveTo { position, speed } => {
            run_damiao_move_to(&cli.interface, &spec, *position as f64, *speed as f64, cli.fd)?;
        }
        Commands::Spin { velocity, duration } => {
            run_damiao_spin(&cli.interface, &spec, *velocity as f64, *duration, cli.fd)?;
        }
        Commands::Torque { torque, duration } => {
            run_damiao_torque(&cli.interface, &spec, *torque as f64, *duration, cli.fd)?;
        }
        Commands::Mit {
            pos,
            vel,
            kp,
            kd,
            torque,
        } => {
            let socket = open_can_socket(&cli.interface, cli.fd)?;
            let mut driver = spec.build();
            driver.enable(&socket)?;
            let fb = driver.mit_exchange(&socket, *pos, *vel, *kp, *kd, *torque)?;
            let _ = driver.disable(&socket);
            print_feedback(&fb);
        }
        Commands::ReadParam { name } => {
            run_damiao_read_param(&cli.interface, &spec, name, cli.fd)?;
        }
        Commands::WriteParam { name, .. } => {
            bail!(
                "DAMIAO write-param '{}' is not supported by the MIT-only protocol implemented in this CLI",
                name
            );
        }
        Commands::Monitor { interval } => {
            run_damiao_monitor(&cli.interface, &spec, *interval, cli.fd)?;
        }
        Commands::Scan { .. }
        | Commands::Dump { .. }
        | Commands::SerialScan { .. }
        | Commands::SerialDump { .. } => unreachable!(),
    }

    Ok(())
}

fn open_can_socket(interface: &str, fd: bool) -> Result<CanBus> {
    let socket = CanBus::open(interface, fd)?;
    socket.set_read_timeout(Duration::from_millis(100))?;
    Ok(socket)
}

fn damiao_feedback_once(interface: &str, spec: &MotorSpec, fd: bool) -> Result<MotorFeedback> {
    let socket = open_can_socket(interface, fd)?;
    let mut driver = spec.build();
    driver.enable(&socket)?;
    let feedback = driver.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0)?;
    let _ = driver.disable(&socket);
    Ok(feedback)
}

fn run_damiao_move_to(interface: &str, spec: &MotorSpec, position: f64, speed: f64, fd: bool) -> Result<()> {
    let socket = open_can_socket(interface, fd)?;
    let mut driver = spec.build();
    let speed = speed.abs();

    driver.enable(&socket)?;
    println!(
        "Moving to {:.3} rad using MIT control (velocity clamp: {:.1} rad/s)",
        position, speed
    );

    loop {
        let fb = driver.mit_exchange(
            &socket,
            position,
            0.0,
            DM_MOVE_KP,
            DM_MOVE_KD,
            0.0,
        )?;
        print!(
            "\r  pos={:.3} vel={:.3} torque={:.3}   ",
            fb.position, fb.velocity, fb.torque
        );

        if (fb.position - position).abs() < 0.05 && fb.velocity.abs() < 0.1 {
            println!("\nTarget reached.");
            break;
        }

        let error = position - fb.position;
        let vel_ref = error.clamp(-speed, speed);
        let _ = driver.mit_exchange(&socket, position, vel_ref, DM_MOVE_KP, DM_MOVE_KD, 0.0);
        std::thread::sleep(DM_CONTROL_PERIOD);
    }

    let _ = driver.disable(&socket);
    Ok(())
}

fn run_damiao_spin(interface: &str, spec: &MotorSpec, velocity: f64, duration: f32, fd: bool) -> Result<()> {
    let socket = open_can_socket(interface, fd)?;
    let mut driver = spec.build();
    let deadline = duration_deadline(duration);

    driver.enable(&socket)?;
    println!("Spinning at {:.2} rad/s via MIT control", velocity);
    if deadline.is_none() {
        println!("Press Ctrl+C to stop.");
    }

    loop {
        let fb = driver.mit_exchange(&socket, 0.0, velocity, 0.0, DM_SPIN_KD, 0.0)?;
        print!(
            "\r  pos={:.3} vel={:.3} torque={:.3}   ",
            fb.position, fb.velocity, fb.torque
        );

        if let Some(deadline) = deadline {
            if Instant::now() >= deadline {
                break;
            }
        }

        std::thread::sleep(DM_CONTROL_PERIOD);
    }

    let _ = driver.mit_exchange(&socket, 0.0, 0.0, 0.0, DM_SPIN_KD, 0.0);
    let _ = driver.disable(&socket);
    println!("\nStopped.");
    Ok(())
}

fn run_damiao_torque(interface: &str, spec: &MotorSpec, torque: f64, duration: f32, fd: bool) -> Result<()> {
    let socket = open_can_socket(interface, fd)?;
    let mut driver = spec.build();
    let deadline = duration_deadline(duration);

    driver.enable(&socket)?;
    println!("Applying torque: {:.3} Nm via MIT control", torque);
    if deadline.is_none() {
        println!("Press Ctrl+C to stop.");
    }

    loop {
        let fb = driver.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, torque)?;
        print!(
            "\r  pos={:.3} vel={:.3} torque={:.3}   ",
            fb.position, fb.velocity, fb.torque
        );

        if let Some(deadline) = deadline {
            if Instant::now() >= deadline {
                break;
            }
        }

        std::thread::sleep(DM_CONTROL_PERIOD);
    }

    let _ = driver.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0);
    let _ = driver.disable(&socket);
    println!("\nStopped.");
    Ok(())
}

fn run_damiao_monitor(interface: &str, spec: &MotorSpec, interval_ms: u64, fd: bool) -> Result<()> {
    let socket = open_can_socket(interface, fd)?;
    let mut driver = spec.build();

    driver.enable(&socket)?;
    println!("Monitoring DAMIAO feedback (interval: {}ms)...", interval_ms);
    println!(
        "{:>10} {:>10} {:>10} {:>8}",
        "Pos[rad]", "Vel[r/s]", "Torq[Nm]", "Temp[°C]"
    );
    println!("{}", "-".repeat(45));

    loop {
        let fb = driver.mit_exchange(&socket, 0.0, 0.0, 0.0, 0.0, 0.0)?;
        print!(
            "\r{:>10.3} {:>10.3} {:>10.3} {:>8.1}",
            fb.position, fb.velocity, fb.torque, fb.temperature
        );
        std::thread::sleep(Duration::from_millis(interval_ms));
    }
}

fn run_damiao_read_param(interface: &str, spec: &MotorSpec, name: &str, fd: bool) -> Result<()> {
    match name.to_lowercase().as_str() {
        "mech_pos" | "position" => {
            let fb = damiao_feedback_once(interface, spec, fd)?;
            println!("{} = {:.4}", name, fb.position);
            Ok(())
        }
        "mech_vel" | "velocity" => {
            let fb = damiao_feedback_once(interface, spec, fd)?;
            println!("{} = {:.4}", name, fb.velocity);
            Ok(())
        }
        "run_mode" | "mode" => {
            println!("{} = mit", name);
            Ok(())
        }
        _ => bail!(
            "DAMIAO read-param '{}' is not available via the MIT-only protocol implemented in this CLI",
            name
        ),
    }
}

fn duration_deadline(duration_secs: f32) -> Option<Instant> {
    if duration_secs > 0.0 {
        Some(Instant::now() + Duration::from_secs_f32(duration_secs))
    } else {
        None
    }
}

fn parse_read_param_name(name: &str) -> Result<ParamIndex> {
    match name.to_lowercase().as_str() {
        "mech_pos" | "position" => Ok(ParamIndex::MechPos),
        "mech_vel" | "velocity" => Ok(ParamIndex::MechVel),
        "iq_filt" | "current" => Ok(ParamIndex::IqFilt),
        "vbus" | "voltage" => Ok(ParamIndex::Vbus),
        "limit_torque" => Ok(ParamIndex::LimitTorque),
        "limit_spd" => Ok(ParamIndex::LimitSpd),
        "limit_cur" => Ok(ParamIndex::LimitCur),
        "run_mode" | "mode" => Ok(ParamIndex::RunMode),
        "loc_kp" => Ok(ParamIndex::LocKp),
        "spd_kp" => Ok(ParamIndex::SpdKp),
        "spd_ki" => Ok(ParamIndex::SpdKi),
        _ => bail!(
            "Unknown parameter '{}'. Available: mech_pos, mech_vel, iq_filt, vbus, limit_torque, limit_spd, limit_cur, run_mode, loc_kp, spd_kp, spd_ki",
            name
        ),
    }
}

fn parse_write_param_name(name: &str) -> Result<ParamIndex> {
    match name.to_lowercase().as_str() {
        "limit_torque" => Ok(ParamIndex::LimitTorque),
        "limit_spd" => Ok(ParamIndex::LimitSpd),
        "limit_cur" => Ok(ParamIndex::LimitCur),
        "loc_kp" => Ok(ParamIndex::LocKp),
        "spd_kp" => Ok(ParamIndex::SpdKp),
        "spd_ki" => Ok(ParamIndex::SpdKi),
        "iq_ref" => Ok(ParamIndex::IqRef),
        "spd_ref" => Ok(ParamIndex::SpdRef),
        "loc_ref" => Ok(ParamIndex::LocRef),
        _ => bail!(
            "Unknown writable parameter '{}'. Available: limit_torque, limit_spd, limit_cur, loc_kp, spd_kp, spd_ki, iq_ref, spd_ref, loc_ref",
            name
        ),
    }
}

fn print_feedback(fb: &MotorFeedback) {
    println!("--- Motor Feedback ---");
    println!("  Motor ID:    {}", fb.motor_id);
    println!("  Position:    {:.4} rad", fb.position);
    println!("  Velocity:    {:.4} rad/s", fb.velocity);
    println!("  Torque:      {:.4} Nm", fb.torque);
    println!("  Temperature: {:.1} C", fb.temperature);
    let s = &fb.status;
    println!("  Mode:        {}", s.mode);
    if s.uncalibrated {
        println!("  WARNING: Encoder uncalibrated");
    }
    if s.stall {
        println!("  WARNING: Motor stall detected");
    }
    if s.magnetic_encoder_fault {
        println!("  WARNING: Magnetic encoder fault");
    }
    if s.overtemperature {
        println!("  WARNING: Over-temperature");
    }
    if s.overcurrent {
        println!("  WARNING: Over-current");
    }
    if s.undervoltage {
        println!("  WARNING: Under-voltage");
    }
}
