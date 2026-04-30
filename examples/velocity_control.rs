//! Example: Velocity control.
//!
//! Spins the motor at different velocities using velocity mode.
//!
//! Usage:
//!   cargo run --example velocity_control

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

    println!("=== Velocity Control Example ===");
    let mut motor = Motor::new(&interface, motor_id, 0xFF, MotorModel::Rs05, false)?;

    // Configure velocity mode
    motor.disable()?;
    motor.set_run_mode(RunMode::Velocity)?;
    motor.set_torque_limit(5.0)?; // 5 Nm torque limit
    motor.enable()?;
    println!("Motor enabled in velocity mode.");

    // Run at different velocities
    let profiles = [
        (2.0_f32, 2.0_f32),   // 2 rad/s for 2 seconds
        (5.0, 2.0),           // 5 rad/s for 2 seconds
        (-3.0, 2.0),          // -3 rad/s for 2 seconds
        (0.0, 1.0),           // stop for 1 second
    ];

    for (vel, duration) in profiles {
        println!("\nSetting velocity to {:.1} rad/s for {:.1}s", vel, duration);
        motor.set_velocity(vel)?;

        let steps = (duration / 0.1) as usize;
        for _ in 0..steps {
            std::thread::sleep(Duration::from_millis(100));
            match motor.read_status() {
                Ok(fb) => {
                    print!(
                        "\r  pos={:.3} vel={:.3} torque={:.3}",
                        fb.position, fb.velocity, fb.torque
                    );
                }
                Err(e) => {
                    print!("\r  read error: {}", e);
                }
            }
        }
        println!();
    }

    motor.set_velocity(0.0)?;
    std::thread::sleep(Duration::from_millis(500));
    motor.disable()?;
    println!("\nDone. Motor disabled.");

    Ok(())
}
