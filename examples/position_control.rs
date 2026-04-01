//! Example: Position control.
//!
//! Moves the motor to a series of positions using position mode.
//!
//! Usage:
//!   cargo run --example position_control

use anyhow::Result;
use robstride_sandbox::motor::Motor;
use robstride_sandbox::protocol::{MotorModel, RunMode};
use std::time::Duration;

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

    println!("=== Position Control Example ===");
    let mut motor = Motor::new(&interface, motor_id, 0xFF, MotorModel::Rs05)?;

    // Configure position mode
    motor.disable()?;
    motor.set_run_mode(RunMode::Position)?;
    motor.set_position_speed_limit(5.0)?; // 5 rad/s max speed
    motor.set_current_limit(3.0)?; // 3 A current limit
    motor.enable()?;
    println!("Motor enabled in position mode.");

    // Move to a series of positions
    let targets = [1.0_f32, 3.0, 0.0, -2.0, 0.0];

    for (i, &target) in targets.iter().enumerate() {
        println!("\n[{}/{}] Moving to {:.2} rad...", i + 1, targets.len(), target);
        motor.set_position(target)?;

        // Wait until position reached
        loop {
            std::thread::sleep(Duration::from_millis(50));
            let fb = motor.read_status()?;
            print!(
                "\r  pos={:.3} vel={:.3} torque={:.3}",
                fb.position, fb.velocity, fb.torque
            );

            if (fb.position - target as f64).abs() < 0.05 && fb.velocity.abs() < 0.1 {
                println!(" ✓");
                break;
            }
        }

        // Brief pause between moves
        std::thread::sleep(Duration::from_millis(500));
    }

    motor.disable()?;
    println!("\nDone. Motor disabled.");

    Ok(())
}
