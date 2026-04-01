//! Robstride motor control CLI.
//!
//! Usage:
//!   robstride_sandbox --interface can0 --motor-id 1 --model rs-05 <COMMAND>

use anyhow::Result;
use clap::{Parser, Subcommand};
use robstride_sandbox::motor::Motor;
use robstride_sandbox::protocol::{
    MotorModel, RunMode, DEFAULT_HOST_ID, parse_can_id,
};
use robstride_sandbox::serial_can::SerialCan;
use std::time::Duration;

#[derive(Parser)]
#[command(name = "robstride_sandbox")]
#[command(about = "Robstride motor controller CLI")]
struct Cli {
    /// CAN interface name (e.g., can0)
    #[arg(short, long, default_value = "can0")]
    interface: String,

    /// Motor CAN ID (1-254)
    #[arg(short, long, default_value_t = 1)]
    motor_id: u8,

    /// Host CAN ID (must be > motor_id, default 0xFF)
    #[arg(long, default_value_t = DEFAULT_HOST_ID)]
    host_id: u8,

    /// Motor model (rs-00..rs-06, edulite05)
    #[arg(long, default_value = "rs-05")]
    model: String,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Ping the motor (GET_DEVICE_ID)
    Ping,

    /// Read motor status
    Status,

    /// Enable the motor
    Enable,

    /// Disable the motor
    Disable,

    /// Set current position as zero
    SetZero,

    /// Move to a position (position mode)
    MoveTo {
        /// Target position in radians
        position: f32,
        /// Speed limit in rad/s
        #[arg(short, long, default_value_t = 5.0)]
        speed: f32,
    },

    /// Set velocity (velocity mode)
    Spin {
        /// Target velocity in rad/s
        velocity: f32,
        /// Duration in seconds (0 = indefinite)
        #[arg(short, long, default_value_t = 0.0)]
        duration: f32,
    },

    /// Apply torque (torque mode)
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
        /// Parameter name (mech_pos, mech_vel, iq_filt, vbus, limit_torque, limit_spd, etc.)
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

    /// Scan the CAN bus for all connected motors
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

    /// Scan motors via direct serial port (bypasses SocketCAN/slcand)
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

    // Scan and Dump don't need a Motor instance
    match cli.command {
        Commands::Scan { from, to, timeout } => {
            println!(
                "Scanning CAN bus '{}' for motors (ID {}..={}, timeout {}ms/id, host=0x{:02X})...",
                cli.interface, from, to, timeout, cli.host_id
            );
            let results = Motor::scan_bus(
                &cli.interface,
                cli.host_id,
                from..=to,
                Duration::from_millis(timeout),
            );

            if results.is_empty() {
                println!("\nNo motors found.");
                println!("\nTroubleshooting:");
                println!("  1. Is the CAN interface up?  ip link show {}", cli.interface);
                println!("  2. Is the bitrate correct?   (Robstride default: 1Mbps)");
                println!("  3. Is the motor powered on?");
                println!("  4. Check wiring (CAN-H, CAN-L, GND)");
                println!("  5. Try passive listen:  cargo run -- -i {} dump", cli.interface);
            } else {
                println!("\nFound {} motor(s):", results.len());
                for (id, data) in &results {
                    match data {
                        Some(d) => {
                            let hex: Vec<String> = d.iter().map(|b| format!("{:02X}", b)).collect();
                            println!("  ID={}: data=[{}]", id, hex.join(" "));
                        }
                        None => {
                            println!("  ID={}: (responded, no data)", id);
                        }
                    }
                }
            }
            return Ok(());
        }

        Commands::Dump { duration } => {
            println!(
                "Listening on '{}' for {:.1}s (passive, no frames sent)...",
                cli.interface, duration
            );
            let frames = Motor::dump_bus(
                &cli.interface,
                Duration::from_secs_f32(duration),
            );

            if frames.is_empty() {
                println!("\nNo CAN frames received.");
            } else {
                println!("\nReceived {} frame(s):", frames.len());
                println!("{:>12}  {:>6} {:>8} {:>4}  {}",
                    "CAN ID", "Type", "Extra", "Dev", "Payload");
                println!("{}", "-".repeat(60));
                for (can_id, data) in &frames {
                    let (ct, extra, dev) = parse_can_id(*can_id);
                    let hex_data: Vec<String> = data.iter().map(|b| format!("{:02X}", b)).collect();
                    println!("  0x{:08X}  {:>5}  0x{:04X}  {:>3}  [{}]",
                        can_id, ct, extra, dev, hex_data.join(" "));
                }
            }
            return Ok(());
        }

        _ => {} // fall through to motor-specific commands
    }

    // SerialScan and SerialDump handled before Motor::new
    match cli.command {
        Commands::SerialScan {
            ref device,
            baud,
            can_bitrate,
            from,
            to,
            timeout,
        } => {
            println!(
                "Serial scan: {} @ {}baud, CAN {}bps, ID {}..={}",
                device, baud, can_bitrate, from, to
            );
            let mut sc = SerialCan::open(device, baud, can_bitrate)
                .map_err(|e| anyhow::anyhow!("Failed to open serial CAN: {}", e))?;

            let results = sc.scan(cli.host_id, from..=to, Duration::from_millis(timeout));

            if results.is_empty() {
                println!("\nNo motors found via serial.");
            } else {
                println!("\nFound {} motor(s):", results.len());
                for (id, feedback) in &results {
                    match feedback {
                        Some(fb) => {
                            println!("  ID={}: pos={:.3} vel={:.3} torque={:.3} temp={:.1}",
                                id, fb.position, fb.velocity, fb.torque, fb.temperature);
                        }
                        None => {
                            println!("  ID={}: (responded but no feedback data)", id);
                        }
                    }
                }
            }
            return Ok(());
        }

        Commands::SerialDump {
            ref device,
            baud,
            duration,
        } => {
            SerialCan::dump_raw_pretty(device, baud, Duration::from_secs_f32(duration));
            return Ok(());
        }

        _ => {} // fall through to motor-specific commands
    }

    // Parse motor model
    let model = MotorModel::from_str(&cli.model).unwrap_or_else(|| {
        eprintln!("Unknown motor model: {}", cli.model);
        eprintln!("Available: rs-00, rs-01, rs-02, rs-03, rs-04, rs-05 (edulite05), rs-06");
        std::process::exit(1);
    });

    let mut motor = Motor::new(&cli.interface, cli.motor_id, cli.host_id, model)?;
    println!(
        "Connected to CAN '{}', motor ID={}, host=0x{:02X}, model={}",
        cli.interface, cli.motor_id, cli.host_id, model
    );

    match cli.command {
        Commands::Ping => {
            match motor.ping() {
                Ok((device_id, uuid)) => {
                    let hex: Vec<String> = uuid.iter().map(|b| format!("{:02X}", b)).collect();
                    println!("Motor responded! device_id=0x{:04X} UUID=[{}]", device_id, hex.join(" "));
                }
                Err(e) => {
                    println!("No response from motor: {}", e);
                }
            }
        }

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
            motor.set_position_speed_limit(speed)?;
            motor.enable()?;
            motor.set_position(position)?;
            println!(
                "Moving to {:.3} rad (speed limit: {:.1} rad/s)",
                position, speed
            );

            // Monitor until near target
            loop {
                std::thread::sleep(Duration::from_millis(100));
                let fb = motor.read_status()?;
                print!(
                    "\r  pos={:.3} vel={:.3} torque={:.3}   ",
                    fb.position, fb.velocity, fb.torque
                );
                if (fb.position - position as f64).abs() < 0.05 && fb.velocity.abs() < 0.1 {
                    println!("\nTarget reached.");
                    break;
                }
            }
        }

        Commands::Spin { velocity, duration } => {
            motor.disable()?;
            motor.set_run_mode(RunMode::Velocity)?;
            motor.enable()?;
            motor.set_velocity(velocity)?;
            println!("Spinning at {:.2} rad/s", velocity);

            if duration > 0.0 {
                std::thread::sleep(Duration::from_secs_f32(duration));
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
            motor.set_torque(torque)?;
            println!("Applying torque: {:.3} Nm", torque);

            if duration > 0.0 {
                std::thread::sleep(Duration::from_secs_f32(duration));
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
            let fb = motor.mit_control(pos, vel, kp, kd, torque)?;
            print_feedback(&fb);
        }

        Commands::ReadParam { name } => {
            use robstride_sandbox::protocol::ParamIndex;
            let param = match name.as_str() {
                "mech_pos" | "position" => ParamIndex::MechPos,
                "mech_vel" | "velocity" => ParamIndex::MechVel,
                "iq_filt" | "current" => ParamIndex::IqFilt,
                "vbus" | "voltage" => ParamIndex::Vbus,
                "limit_torque" => ParamIndex::LimitTorque,
                "limit_spd" => ParamIndex::LimitSpd,
                "limit_cur" => ParamIndex::LimitCur,
                "run_mode" | "mode" => ParamIndex::RunMode,
                "loc_kp" => ParamIndex::LocKp,
                "spd_kp" => ParamIndex::SpdKp,
                "spd_ki" => ParamIndex::SpdKi,
                _ => {
                    eprintln!("Unknown parameter: {}", name);
                    eprintln!("Available: mech_pos, mech_vel, iq_filt, vbus, limit_torque,");
                    eprintln!("           limit_spd, limit_cur, run_mode, loc_kp, spd_kp, spd_ki");
                    std::process::exit(1);
                }
            };
            let value = motor.read_param(param)?;
            println!("{} = {:.4}", name, value);
        }

        Commands::WriteParam { name, value } => {
            use robstride_sandbox::protocol::ParamIndex;
            let param = match name.as_str() {
                "limit_torque" => ParamIndex::LimitTorque,
                "limit_spd" => ParamIndex::LimitSpd,
                "limit_cur" => ParamIndex::LimitCur,
                "loc_kp" => ParamIndex::LocKp,
                "spd_kp" => ParamIndex::SpdKp,
                "spd_ki" => ParamIndex::SpdKi,
                "iq_ref" => ParamIndex::IqRef,
                "spd_ref" => ParamIndex::SpdRef,
                "loc_ref" => ParamIndex::LocRef,
                _ => {
                    eprintln!("Unknown writable parameter: {}", name);
                    eprintln!("Available: limit_torque, limit_spd, limit_cur, loc_kp, spd_kp, spd_ki,");
                    eprintln!("           iq_ref, spd_ref, loc_ref");
                    std::process::exit(1);
                }
            };
            motor.write_param_f32(param, value)?;
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
                std::thread::sleep(Duration::from_millis(interval));
            }
        }

        // Scan, Dump, SerialScan, SerialDump already handled above
        Commands::Scan { .. }
        | Commands::Dump { .. }
        | Commands::SerialScan { .. }
        | Commands::SerialDump { .. } => unreachable!(),
    }

    Ok(())
}

fn print_feedback(fb: &robstride_sandbox::protocol::MotorFeedback) {
    println!("--- Motor Feedback ---");
    println!("  Motor ID:    {}", fb.motor_id);
    println!("  Position:    {:.4} rad", fb.position);
    println!("  Velocity:    {:.4} rad/s", fb.velocity);
    println!("  Torque:      {:.4} Nm", fb.torque);
    println!("  Temperature: {:.1} °C", fb.temperature);
    let s = &fb.status;
    println!("  Mode:        {}", s.mode);
    if s.uncalibrated { println!("  WARNING: Encoder uncalibrated"); }
    if s.stall { println!("  WARNING: Motor stall detected"); }
    if s.magnetic_encoder_fault { println!("  WARNING: Magnetic encoder fault"); }
    if s.overtemperature { println!("  WARNING: Over-temperature"); }
    if s.overcurrent { println!("  WARNING: Over-current"); }
    if s.undervoltage { println!("  WARNING: Under-voltage"); }
}
