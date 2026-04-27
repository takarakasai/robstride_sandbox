//! Error types for Robstride motor communication.

use thiserror::Error;

/// Errors that can occur during motor communication.
#[derive(Debug, Error)]
pub enum RobstrideError {
    #[error("CAN socket error: {0}")]
    CanSocket(#[from] std::io::Error),

    #[error("Timeout waiting for motor response (motor_id={motor_id})")]
    Timeout { motor_id: u8 },

    #[error("Invalid response from motor: {msg}")]
    InvalidResponse { msg: String },

    #[error("Motor fault detected: code=0x{code:02X}")]
    MotorFault { code: u8 },

    #[error("Parameter value out of range: {param} = {value} (expected {min}..{max})")]
    OutOfRange {
        param: &'static str,
        value: f32,
        min: f32,
        max: f32,
    },

    #[error("Motor not enabled (motor_id={motor_id})")]
    NotEnabled { motor_id: u8 },

    #[error("Serial port error: {0}")]
    SerialPort(String),

    /// Catch-all for misuse / setup errors that don't have a dedicated
    /// variant (e.g. CAN interface lock conflicts, configuration sanity
    /// failures). The message is the canonical surface — log it directly.
    #[error("{0}")]
    Other(String),
}

pub type Result<T> = std::result::Result<T, RobstrideError>;
