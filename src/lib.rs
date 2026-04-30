//! Robstride CAN bus motor controller library
//!
//! This crate provides a Rust interface to control Robstride motors
//! over CAN bus using Linux SocketCAN.
//!
//! # Architecture
//!
//! - [`protocol`] - CAN frame encoding/decoding for the Robstride protocol
//! - [`motor`] - High-level motor control API
//! - [`error`] - Error types
//! - [`serial_can`] - Direct serial SLCAN transport (debugging)
//!
//! # Quick Start
//!
//! ```no_run
//! use robstride_sandbox::{motor::Motor, protocol::{MotorModel, RunMode}};
//!
//! let mut motor = Motor::new("can0", 1, 0xFF, MotorModel::Rs05, false)?;
//! motor.enable()?;
//! motor.set_run_mode(RunMode::Position)?;
//! motor.set_position(3.14)?;
//! let status = motor.read_status()?;
//! println!("Position: {:.3} rad", status.position);
//! motor.disable()?;
//! # Ok::<(), anyhow::Error>(())
//! ```

pub mod bilateral;
pub mod can_lock;
pub mod driver;
pub mod error;
pub mod motor;
pub mod protocol;
pub mod serial_can;
pub mod transport;
