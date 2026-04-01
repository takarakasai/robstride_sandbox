//! Example: Read motor status.
//!
//! Usage:
//!   cargo run --example status -- [--interface can0] [--motor-id 1]

use anyhow::Result;
use robstride_sandbox::motor::Motor;
use robstride_sandbox::protocol::MotorModel;

fn main() -> Result<()> {
    env_logger::init();

    let interface = std::env::args()
        .position(|a| a == "--interface")
        .and_then(|i| std::env::args().nth(i + 1))
        .unwrap_or_else(|| "can0".to_string());

    let motor_id: u8 = std::env::args()
        .position(|a| a == "--motor-id")
        .and_then(|i| std::env::args().nth(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(1);

    println!("Connecting to motor {} on {}", motor_id, interface);
    let motor = Motor::new(&interface, motor_id, 0xFF, MotorModel::Rs05)?;

    // Read status via MIT zero-command
    match motor.read_status() {
        Ok(fb) => {
            println!("Motor Status:");
            println!("  Position:    {:.4} rad ({:.2}°)", fb.position, fb.position.to_degrees());
            println!("  Velocity:    {:.4} rad/s", fb.velocity);
            println!("  Torque:      {:.4} Nm", fb.torque);
            println!("  Temperature: {:.1} °C", fb.temperature);
            println!("  Mode:        {}", fb.status.mode);
        }
        Err(e) => {
            eprintln!("Failed to read status: {}", e);
        }
    }

    // Also try reading individual parameters
    println!("\nParameter reads:");
    for (name, param) in [
        ("Mech Position", robstride_sandbox::protocol::ParamIndex::MechPos),
        ("Mech Velocity", robstride_sandbox::protocol::ParamIndex::MechVel),
        ("Bus Voltage", robstride_sandbox::protocol::ParamIndex::Vbus),
    ] {
        match motor.read_param(param) {
            Ok(val) => println!("  {}: {:.4}", name, val),
            Err(e) => println!("  {}: error - {}", name, e),
        }
    }

    Ok(())
}
