//! Robstride CAN protocol implementation.
//!
//! Based on the official Robstride protocol specification.
//! The protocol uses CAN extended frames (29-bit ID).
//!
//! ## CAN Extended ID structure (29 bits):
//! ```text
//! | Bits 28-24 (5) | Bits 23-8 (16) | Bits 7-0 (8) |
//! | Comm Type      | Extra Data     | Device ID    |
//! ```
//!
//! ## Communication types:
//! - 0: GET_DEVICE_ID - Get device ID and MCU UUID
//! - 1: OPERATION_CONTROL - MIT mode control (pos/vel/kp/kd in data, torque in extra_data)
//! - 2: OPERATION_STATUS - Motor status feedback
//! - 3: ENABLE - Enable motor
//! - 4: DISABLE - Disable motor
//! - 6: SET_ZERO_POSITION - Set current position as zero
//! - 7: SET_DEVICE_ID - Change motor CAN ID
//! - 17: READ_PARAMETER - Read a parameter
//! - 18: WRITE_PARAMETER - Write a parameter
//! - 21: FAULT_REPORT - Fault report
//! - 22: SAVE_PARAMETERS - Save parameters to flash
//! - 23: SET_BAUDRATE - Set CAN baudrate
//! - 24: ACTIVE_REPORT - Motor active report
//! - 25: SET_PROTOCOL - Set protocol type

use std::f64::consts::PI;

/// Default host ID. Must be greater than any motor ID for optimal performance.
pub const DEFAULT_HOST_ID: u8 = 0xFF;

// =============================================================================
// Communication types
// =============================================================================

/// Communication type codes in the Robstride CAN protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CommType {
    /// Get device ID and 64-bit MCU unique identifier
    GetDeviceId = 0,
    /// MIT mode operation control (pos/vel/kp/kd + torque feedforward)
    OperationControl = 1,
    /// Motor status feedback frame
    OperationStatus = 2,
    /// Enable the motor
    Enable = 3,
    /// Disable the motor
    Disable = 4,
    /// Set current position as mechanical zero
    SetZeroPosition = 6,
    /// Set device CAN ID
    SetDeviceId = 7,
    /// Read a parameter
    ReadParameter = 17,
    /// Write a parameter
    WriteParameter = 18,
    /// Fault report feedback
    FaultReport = 21,
    /// Save all parameters to flash
    SaveParameters = 22,
    /// Set CAN baudrate
    SetBaudrate = 23,
    /// Motor active report
    ActiveReport = 24,
    /// Set protocol type
    SetProtocol = 25,
}

/// Motor run modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RunMode {
    /// MIT mode (direct position/velocity/torque with gains)
    Mit = 0,
    /// Position mode
    Position = 1,
    /// Velocity mode
    Velocity = 2,
    /// Torque (current) mode
    Torque = 3,
}

// =============================================================================
// Motor model definitions and scaling tables
// =============================================================================

/// Motor model identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorModel {
    Rs00,
    Rs01,
    Rs02,
    Rs03,
    Rs04,
    Rs05,
    Rs06,
}

impl MotorModel {
    /// Parse a model name string.
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "rs-00" | "rs00" => Some(MotorModel::Rs00),
            "rs-01" | "rs01" | "edulite01" => Some(MotorModel::Rs01),
            "rs-02" | "rs02" | "edulite02" => Some(MotorModel::Rs02),
            "rs-03" | "rs03" => Some(MotorModel::Rs03),
            "rs-04" | "rs04" => Some(MotorModel::Rs04),
            "rs-05" | "rs05" | "edulite05" => Some(MotorModel::Rs05),
            "rs-06" | "rs06" => Some(MotorModel::Rs06),
            _ => None,
        }
    }

    /// Display name
    pub fn name(&self) -> &'static str {
        match self {
            MotorModel::Rs00 => "RS-00",
            MotorModel::Rs01 => "RS-01",
            MotorModel::Rs02 => "RS-02",
            MotorModel::Rs03 => "RS-03",
            MotorModel::Rs04 => "RS-04",
            MotorModel::Rs05 => "RS-05",
            MotorModel::Rs06 => "RS-06",
        }
    }
}

impl std::fmt::Display for MotorModel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

/// MIT mode scaling parameters for a specific motor model.
#[derive(Debug, Clone, Copy)]
pub struct MitScales {
    /// Position scale [rad] (range is -scale..+scale)
    pub position: f64,
    /// Velocity scale [rad/s] (range is -scale..+scale)
    pub velocity: f64,
    /// Torque scale [Nm] (range is -scale..+scale)
    pub torque: f64,
    /// Kp scale (range is 0..scale)
    pub kp: f64,
    /// Kd scale (range is 0..scale)
    pub kd: f64,
}

impl MitScales {
    /// Get the MIT scaling table for a given motor model.
    pub fn for_model(model: MotorModel) -> Self {
        match model {
            MotorModel::Rs00 => MitScales {
                position: 4.0 * PI,
                velocity: 50.0,
                torque: 17.0,
                kp: 500.0,
                kd: 5.0,
            },
            MotorModel::Rs01 => MitScales {
                position: 4.0 * PI,
                velocity: 44.0,
                torque: 17.0,
                kp: 500.0,
                kd: 5.0,
            },
            MotorModel::Rs02 => MitScales {
                position: 4.0 * PI,
                velocity: 44.0,
                torque: 17.0,
                kp: 500.0,
                kd: 5.0,
            },
            MotorModel::Rs03 => MitScales {
                position: 4.0 * PI,
                velocity: 50.0,
                torque: 60.0,
                kp: 5000.0,
                kd: 100.0,
            },
            MotorModel::Rs04 => MitScales {
                position: 4.0 * PI,
                velocity: 15.0,
                torque: 120.0,
                kp: 5000.0,
                kd: 100.0,
            },
            MotorModel::Rs05 => MitScales {
                position: 4.0 * PI,
                velocity: 33.0,
                torque: 17.0,
                kp: 500.0,
                kd: 5.0,
            },
            MotorModel::Rs06 => MitScales {
                position: 4.0 * PI,
                velocity: 20.0,
                torque: 60.0,
                kp: 5000.0,
                kd: 100.0,
            },
        }
    }
}

// =============================================================================
// Motor status
// =============================================================================

/// Status bits decoded from the extra_data field of a status response.
#[derive(Debug, Clone, Copy, Default)]
pub struct MotorStatusBits {
    /// Motor mode (2 bits)
    pub mode: u8,
    /// Encoder uncalibrated
    pub uncalibrated: bool,
    /// Motor stalled
    pub stall: bool,
    /// Magnetic encoder fault
    pub magnetic_encoder_fault: bool,
    /// Over-temperature
    pub overtemperature: bool,
    /// Over-current
    pub overcurrent: bool,
    /// Under-voltage
    pub undervoltage: bool,
    /// Device ID from the extra_data field
    pub device_id: u8,
}

/// Motor feedback data decoded from CAN response.
#[derive(Debug, Clone)]
pub struct MotorFeedback {
    /// Motor CAN ID (from the CAN frame's device_id field)
    pub motor_id: u8,
    /// Current position in radians
    pub position: f64,
    /// Current velocity in rad/s
    pub velocity: f64,
    /// Current torque in Nm
    pub torque: f64,
    /// Motor temperature in °C
    pub temperature: f64,
    /// Status bits
    pub status: MotorStatusBits,
}

// =============================================================================
// Parameter definitions
// =============================================================================

/// Robstride parameter indices for read/write commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum ParamIndex {
    MechOffset = 0x2005,
    MeasuredPosition = 0x3016,
    MeasuredVelocity = 0x3017,
    MeasuredTorque = 0x302C,
    RunMode = 0x7005,
    IqRef = 0x7006,
    SpdRef = 0x700A,
    LimitTorque = 0x700B,
    CurKp = 0x7010,
    CurKi = 0x7011,
    CurFiltGain = 0x7014,
    LocRef = 0x7016,
    LimitSpd = 0x7017,
    LimitCur = 0x7018,
    MechPos = 0x7019,
    IqFilt = 0x701A,
    MechVel = 0x701B,
    Vbus = 0x701C,
    LocKp = 0x701E,
    SpdKp = 0x701F,
    SpdKi = 0x7020,
    SpdFiltGain = 0x7021,
    AccRad = 0x7022,
    VelMax = 0x7024,
    AccSet = 0x7025,
    CanTimeout = 0x7028,
    ZeroState = 0x7029,
}

// =============================================================================
// CAN ID encoding / decoding
// =============================================================================

/// Build a 29-bit extended CAN ID.
///
/// Layout: `[comm_type(5 bits)] [extra_data(16 bits)] [device_id(8 bits)]`
///
/// - `comm_type`: Communication type (bits 28-24)
/// - `extra_data`: Secondary data field (bits 23-8), typically host_id or torque
/// - `device_id`: Target motor ID (bits 7-0)
pub fn build_can_id(comm_type: CommType, extra_data: u16, device_id: u8) -> u32 {
    ((comm_type as u32) << 24)
        | ((extra_data as u32) << 8)
        | (device_id as u32)
}

/// Decode a 29-bit extended CAN ID.
///
/// Returns (comm_type, extra_data, device_id).
pub fn parse_can_id(id: u32) -> (u8, u16, u8) {
    let comm_type = ((id >> 24) & 0x1F) as u8;
    let extra_data = ((id >> 8) & 0xFFFF) as u16;
    let device_id = (id & 0xFF) as u8;
    (comm_type, extra_data, device_id)
}

// =============================================================================
// MIT mode value encoding / decoding
// =============================================================================

/// Encode a signed value (position, velocity, torque) for MIT mode.
///
/// Maps [-scale, +scale] to [0, 0xFFFF] with 0x7FFF at zero.
/// Formula: `((value / scale) + 1.0) * 0x7FFF`
pub fn encode_mit_signed(value: f64, scale: f64) -> u16 {
    let clamped = value.max(-scale).min(scale);
    let u = ((clamped / scale) + 1.0) * 0x7FFF as f64;
    (u as u16).min(0xFFFF)
}

/// Decode a signed MIT value back to float.
///
/// Formula: `(u16 / 0x7FFF - 1.0) * scale`
pub fn decode_mit_signed(raw: u16, scale: f64) -> f64 {
    ((raw as f64) / 0x7FFF as f64 - 1.0) * scale
}

/// Encode an unsigned value (kp, kd) for MIT mode.
///
/// Maps [0, scale] to [0, 0xFFFF].
/// Formula: `(value / scale) * 0xFFFF`
pub fn encode_mit_unsigned(value: f64, scale: f64) -> u16 {
    let clamped = value.max(0.0).min(scale);
    ((clamped / scale) * 0xFFFF as f64) as u16
}

/// Decode an unsigned MIT value back to float.
///
/// Formula: `(u16 / 0xFFFF) * scale`
pub fn decode_mit_unsigned(raw: u16, scale: f64) -> f64 {
    (raw as f64 / 0xFFFF as f64) * scale
}

// =============================================================================
// Frame builders
// =============================================================================

/// Build a GET_DEVICE_ID (ping) frame.
///
/// Used to probe whether a motor is present on the bus.
pub fn build_ping_frame(host_id: u8, device_id: u8) -> (u32, Vec<u8>) {
    let can_id = build_can_id(CommType::GetDeviceId, host_id as u16, device_id);
    (can_id, vec![0u8; 8])
}

/// Build a motor enable frame.
pub fn build_enable_frame(host_id: u8, device_id: u8) -> (u32, Vec<u8>) {
    let can_id = build_can_id(CommType::Enable, host_id as u16, device_id);
    (can_id, vec![])
}

/// Build a motor disable frame.
pub fn build_disable_frame(host_id: u8, device_id: u8) -> (u32, Vec<u8>) {
    let can_id = build_can_id(CommType::Disable, host_id as u16, device_id);
    (can_id, vec![0u8; 8])
}

/// Build a "set zero position" frame.
pub fn build_set_zero_frame(host_id: u8, device_id: u8) -> (u32, Vec<u8>) {
    let can_id = build_can_id(CommType::SetZeroPosition, host_id as u16, device_id);
    (can_id, vec![1u8, 0, 0, 0, 0, 0, 0, 0])
}

/// Build a MIT-mode operation control frame.
///
/// Data field (big-endian): [pos_u16, vel_u16, kp_u16, kd_u16]
/// Torque is encoded in the extra_data field of the CAN ID (16 bits).
/// Note: host_id is not used in this frame - extra_data carries the torque value.
pub fn build_mit_frame(
    _host_id: u8,
    device_id: u8,
    scales: &MitScales,
    position: f64,
    velocity: f64,
    kp: f64,
    kd: f64,
    torque: f64,
) -> (u32, Vec<u8>) {
    let pos_u16 = encode_mit_signed(position, scales.position);
    let vel_u16 = encode_mit_signed(velocity, scales.velocity);
    let kp_u16 = encode_mit_unsigned(kp, scales.kp);
    let kd_u16 = encode_mit_unsigned(kd, scales.kd);
    let torque_u16 = encode_mit_signed(torque, scales.torque);

    let can_id = build_can_id(CommType::OperationControl, torque_u16, device_id);

    let data = vec![
        (pos_u16 >> 8) as u8,
        (pos_u16 & 0xFF) as u8,
        (vel_u16 >> 8) as u8,
        (vel_u16 & 0xFF) as u8,
        (kp_u16 >> 8) as u8,
        (kp_u16 & 0xFF) as u8,
        (kd_u16 >> 8) as u8,
        (kd_u16 & 0xFF) as u8,
    ];

    (can_id, data)
}

/// Build a read parameter frame.
pub fn build_read_param_frame(host_id: u8, device_id: u8, param: ParamIndex) -> (u32, Vec<u8>) {
    let can_id = build_can_id(CommType::ReadParameter, host_id as u16, device_id);
    let idx = param as u16;
    let mut data = vec![0u8; 8];
    data[0] = (idx & 0xFF) as u8;
    data[1] = (idx >> 8) as u8;
    (can_id, data)
}

/// Build a write parameter frame (f32 value).
pub fn build_write_param_f32_frame(
    host_id: u8,
    device_id: u8,
    param: ParamIndex,
    value: f32,
) -> (u32, Vec<u8>) {
    let can_id = build_can_id(CommType::WriteParameter, host_id as u16, device_id);
    let idx = param as u16;
    let val_bytes = value.to_le_bytes();
    let mut data = vec![0u8; 8];
    data[0] = (idx & 0xFF) as u8;
    data[1] = (idx >> 8) as u8;
    // bytes 2-3 are reserved (0x00)
    data[4] = val_bytes[0];
    data[5] = val_bytes[1];
    data[6] = val_bytes[2];
    data[7] = val_bytes[3];
    (can_id, data)
}

/// Build a write parameter frame (i8 value, for run mode etc.).
pub fn build_write_param_i8_frame(
    host_id: u8,
    device_id: u8,
    param: ParamIndex,
    value: i8,
) -> (u32, Vec<u8>) {
    let can_id = build_can_id(CommType::WriteParameter, host_id as u16, device_id);
    let idx = param as u16;
    let mut data = vec![0u8; 8];
    data[0] = (idx & 0xFF) as u8;
    data[1] = (idx >> 8) as u8;
    data[4] = value as u8;
    (can_id, data)
}

/// Build a run mode change frame.
pub fn build_run_mode_frame(host_id: u8, device_id: u8, mode: RunMode) -> (u32, Vec<u8>) {
    build_write_param_i8_frame(host_id, device_id, ParamIndex::RunMode, mode as i8)
}

// =============================================================================
// Response parsing
// =============================================================================

/// Parse the extra_data field from an OPERATION_STATUS response.
fn parse_status_bits(extra_data: u16) -> MotorStatusBits {
    MotorStatusBits {
        mode: ((extra_data >> 14) & 0x03) as u8,
        uncalibrated: ((extra_data >> 13) & 0x01) != 0,
        stall: ((extra_data >> 12) & 0x01) != 0,
        magnetic_encoder_fault: ((extra_data >> 11) & 0x01) != 0,
        overtemperature: ((extra_data >> 10) & 0x01) != 0,
        overcurrent: ((extra_data >> 9) & 0x01) != 0,
        undervoltage: ((extra_data >> 8) & 0x01) != 0,
        device_id: (extra_data & 0xFF) as u8,
    }
}

/// Parse a motor status feedback frame (CommType::OperationStatus = 2).
///
/// The data bytes are big-endian: [pos_u16, vel_u16, torque_u16, temp_u16].
/// Decoding uses the signed MIT formula for pos/vel/torque.
pub fn parse_status_frame(
    can_id: u32,
    data: &[u8],
    scales: &MitScales,
) -> Option<MotorFeedback> {
    if data.len() < 8 {
        return None;
    }

    let (comm_type, extra_data, _device_id) = parse_can_id(can_id);

    // Only parse status frames (type 2) or fault frames (type 21)
    if comm_type != CommType::OperationStatus as u8 && comm_type != CommType::FaultReport as u8 {
        return None;
    }

    let status = parse_status_bits(extra_data);

    // Data: big-endian u16 fields
    let pos_u16 = ((data[0] as u16) << 8) | (data[1] as u16);
    let vel_u16 = ((data[2] as u16) << 8) | (data[3] as u16);
    let torque_u16 = ((data[4] as u16) << 8) | (data[5] as u16);
    let temp_u16 = ((data[6] as u16) << 8) | (data[7] as u16);

    let position = decode_mit_signed(pos_u16, scales.position);
    let velocity = decode_mit_signed(vel_u16, scales.velocity);
    let torque = decode_mit_signed(torque_u16, scales.torque);
    let temperature = temp_u16 as f64 * 0.1;

    Some(MotorFeedback {
        motor_id: status.device_id,
        position,
        velocity,
        torque,
        temperature,
        status,
    })
}

/// Parse a parameter read response.
///
/// Returns (param_index, raw 4 bytes) from data[4..8].
pub fn parse_param_response(data: &[u8]) -> Option<(u16, f32)> {
    if data.len() < 8 {
        return None;
    }
    let idx = (data[0] as u16) | ((data[1] as u16) << 8);
    let val = f32::from_le_bytes([data[4], data[5], data[6], data[7]]);
    Some((idx, val))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_can_id_roundtrip() {
        let id = build_can_id(CommType::OperationControl, 0x7FFF, 0x01);
        let (ct, extra, dev) = parse_can_id(id);
        assert_eq!(ct, CommType::OperationControl as u8);
        assert_eq!(extra, 0x7FFF);
        assert_eq!(dev, 0x01);
    }

    #[test]
    fn test_can_id_enable() {
        // Enable motor ID 1 from host 0xFF
        let id = build_can_id(CommType::Enable, 0xFF, 0x01);
        assert_eq!(id, (3 << 24) | (0xFF << 8) | 1);
        let (ct, extra, dev) = parse_can_id(id);
        assert_eq!(ct, 3);
        assert_eq!(extra, 0xFF);
        assert_eq!(dev, 1);
    }

    #[test]
    fn test_mit_signed_zero() {
        // Zero should encode to 0x7FFF
        let encoded = encode_mit_signed(0.0, 4.0 * PI);
        assert_eq!(encoded, 0x7FFF);
    }

    #[test]
    fn test_mit_signed_roundtrip() {
        let scale = 4.0 * PI;
        let val = 1.5;
        let encoded = encode_mit_signed(val, scale);
        let decoded = decode_mit_signed(encoded, scale);
        assert!((val - decoded).abs() < 0.01, "got {}", decoded);
    }

    #[test]
    fn test_mit_unsigned_roundtrip() {
        let scale = 500.0;
        let val = 100.0;
        let encoded = encode_mit_unsigned(val, scale);
        let decoded = decode_mit_unsigned(encoded, scale);
        assert!((val - decoded).abs() < 0.01, "got {}", decoded);
    }

    #[test]
    fn test_mit_signed_boundaries() {
        let scale = 50.0;
        // -scale -> 0x0000
        assert_eq!(encode_mit_signed(-50.0, scale), 0);
        // +scale -> 0xFFFE (close to 0xFFFF)
        let max_encoded = encode_mit_signed(50.0, scale);
        assert!(max_encoded >= 0xFFFE, "got 0x{:04X}", max_encoded);
    }

    #[test]
    fn test_status_bits_parsing() {
        // Example extra_data with device_id=1, undervoltage set
        let extra = (1 << 8) | 1; // undervoltage + device_id=1
        let bits = parse_status_bits(extra);
        assert!(bits.undervoltage);
        assert_eq!(bits.device_id, 1);
        assert!(!bits.overcurrent);
    }
}
