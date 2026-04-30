//! Example: Torque control.
//!
//! Applies torque profiles using torque (current) mode.
//!
//! Usage:
//!   cargo run --example torque_control

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

    println!("=== Torque Control Example ===");
    println!("WARNING: The motor will apply torque. Ensure it is safe to do so!");
    println!("Starting in 3 seconds...");
    std::thread::sleep(Duration::from_secs(3));

    let mut motor = Motor::new(&interface, motor_id, 0xFF, MotorModel::Rs05, false)?;

    // Configure torque mode
    motor.disable()?;
    motor.set_run_mode(RunMode::Torque)?;
    motor.enable()?;
    println!("Motor enabled in torque mode.");

    // Apply a gentle sinusoidal torque
    let duration_secs = 5.0;
    let freq_hz = 0.5;
    let amplitude = 0.5; // 0.5 Nm
    let dt = 0.01; // 10ms control loop
    let steps = (duration_secs / dt) as usize;

    println!("Applying sinusoidal torque: {:.2} Nm @ {:.1} Hz for {:.0}s",
        amplitude, freq_hz, duration_secs);

    for i in 0..steps {
        let t = i as f32 * dt;
        let torque = amplitude * (2.0 * std::f32::consts::PI * freq_hz * t).sin();

        motor.set_torque(torque)?;

        if i % 10 == 0 {
            match motor.read_status() {
                Ok(fb) => {
                    print!(
                        "\r  t={:.2}s cmd={:.3}Nm pos={:.3} vel={:.3}",
                        t, torque, fb.position, fb.velocity
                    );
                }
                Err(_) => {}
            }
        }

        std::thread::sleep(Duration::from_secs_f32(dt));
    }

    motor.set_torque(0.0)?;
    std::thread::sleep(Duration::from_millis(100));
    motor.disable()?;
    println!("\n\nDone. Motor disabled.");

    Ok(())
}
