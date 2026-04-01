//! High-level motor control API for Robstride motors.
//!
//! Provides a safe, ergonomic interface over the raw CAN protocol.
//! Based on the official Robstride protocol specification.

use std::time::{Duration, Instant};

use socketcan::{CanSocket, EmbeddedFrame, ExtendedId, Id, Socket, StandardId};

use crate::error::{Result, RobstrideError};
use crate::protocol::*;

/// Default timeout for waiting for a motor response.
const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);

/// High-level controller for a single Robstride motor.
pub struct Motor {
    socket: CanSocket,
    /// CAN ID of the target motor (1-254)
    motor_id: u8,
    /// Host CAN ID (default 0xFF)
    host_id: u8,
    /// Motor model (determines MIT scaling)
    model: MotorModel,
    /// MIT scaling parameters
    scales: MitScales,
    /// Whether the motor is currently enabled
    enabled: bool,
    /// Current run mode
    run_mode: RunMode,
    /// Response timeout
    timeout: Duration,
}

impl Motor {
    /// Create a new Motor instance connected to the given CAN interface.
    ///
    /// # Arguments
    /// * `interface` - SocketCAN interface name (e.g., "can0")
    /// * `motor_id` - CAN ID of the target motor (1-254)
    /// * `host_id` - Host CAN ID (default 0xFF, must be > motor_id)
    /// * `model` - Motor model for MIT scaling
    pub fn new(interface: &str, motor_id: u8, host_id: u8, model: MotorModel) -> Result<Self> {
        let socket = CanSocket::open(interface)?;
        socket.set_read_timeout(DEFAULT_TIMEOUT)?;

        let scales = MitScales::for_model(model);

        Ok(Motor {
            socket,
            motor_id,
            host_id,
            model,
            scales,
            enabled: false,
            run_mode: RunMode::Mit,
            timeout: DEFAULT_TIMEOUT,
        })
    }

    /// Set the response timeout.
    pub fn set_timeout(&mut self, timeout: Duration) {
        self.timeout = timeout;
    }

    /// Get the motor CAN ID.
    pub fn motor_id(&self) -> u8 {
        self.motor_id
    }

    /// Get the motor model.
    pub fn model(&self) -> MotorModel {
        self.model
    }

    /// Get the MIT scales.
    pub fn scales(&self) -> &MitScales {
        &self.scales
    }

    /// Check if the motor is enabled.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Get the current run mode.
    pub fn current_run_mode(&self) -> RunMode {
        self.run_mode
    }

    // =========================================================================
    // Low-level CAN send/receive
    // =========================================================================

    /// Send a CAN frame with extended ID and variable-length data.
    fn send_frame(&self, can_id: u32, data: &[u8]) -> Result<()> {
        let ext_id = ExtendedId::new(can_id).expect("Invalid extended CAN ID");
        let frame = socketcan::CanFrame::new(ext_id, data).expect("Failed to create CAN frame");
        self.socket.write_frame(&frame)?;
        log::debug!(
            "TX: CAN ID=0x{:08X} data={:02X?}",
            can_id,
            data,
        );
        Ok(())
    }

    /// Receive a CAN frame, with timeout.
    ///
    /// Returns (comm_type, extra_data, device_id, data).
    fn recv_frame(&self) -> Result<(u8, u16, u8, Vec<u8>)> {
        let start = Instant::now();
        loop {
            if start.elapsed() > self.timeout {
                return Err(RobstrideError::Timeout {
                    motor_id: self.motor_id,
                });
            }

            match self.socket.read_frame() {
                Ok(frame) => {
                    if !frame.is_extended() {
                        // Non-extended frames can occur when motor reconnects (type 0)
                        // Skip them
                        continue;
                    }

                    let raw_id = match frame.id() {
                        Id::Standard(sid) => StandardId::as_raw(&sid) as u32,
                        Id::Extended(eid) => ExtendedId::as_raw(&eid),
                    };

                    let data = frame.data().to_vec();
                    let (comm_type, extra_data, device_id) = parse_can_id(raw_id);

                    log::debug!(
                        "RX: CAN ID=0x{:08X} comm={} extra=0x{:04X} dev={} data={:02X?}",
                        raw_id, comm_type, extra_data, device_id, &data,
                    );

                    return Ok((comm_type, extra_data, device_id, data));
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    continue;
                }
                Err(e) => return Err(RobstrideError::CanSocket(e)),
            }
        }
    }

    /// Receive a status frame from our motor.
    fn recv_status(&self) -> Result<MotorFeedback> {
        let (comm_type, extra_data, device_id, data) = self.recv_frame()?;

        if comm_type == CommType::FaultReport as u8 {
            return Err(RobstrideError::MotorFault {
                code: comm_type,
            });
        }

        if comm_type != CommType::OperationStatus as u8 {
            return Err(RobstrideError::InvalidResponse {
                msg: format!(
                    "Expected OperationStatus (type=2), got type={} from device={}",
                    comm_type, device_id
                ),
            });
        }

        parse_status_frame(
            build_can_id_raw(comm_type, extra_data, device_id),
            &data,
            &self.scales,
        )
        .ok_or(RobstrideError::InvalidResponse {
            msg: "Failed to parse status frame".into(),
        })
    }

    // =========================================================================
    // Motor control commands
    // =========================================================================

    /// Ping the motor (GET_DEVICE_ID).
    ///
    /// Returns Some((device_id, uuid_bytes)) if the motor responds.
    pub fn ping(&self) -> Result<(u16, Vec<u8>)> {
        let (can_id, data) = build_ping_frame(self.host_id, self.motor_id);
        self.send_frame(can_id, &data)?;
        let (_comm_type, extra_data, _device_id, resp_data) = self.recv_frame()?;
        Ok((extra_data, resp_data))
    }

    /// Enable the motor.
    ///
    /// Must be called before any motion commands.
    pub fn enable(&mut self) -> Result<MotorFeedback> {
        let (can_id, data) = build_enable_frame(self.host_id, self.motor_id);
        self.send_frame(can_id, &data)?;
        let fb = self.recv_status()?;
        self.enabled = true;
        Ok(fb)
    }

    /// Disable the motor (coast to stop).
    pub fn disable(&mut self) -> Result<MotorFeedback> {
        let (can_id, data) = build_disable_frame(self.host_id, self.motor_id);
        self.send_frame(can_id, &data)?;
        let fb = self.recv_status()?;
        self.enabled = false;
        Ok(fb)
    }

    /// Set the current position as mechanical zero.
    pub fn set_zero(&mut self) -> Result<()> {
        let (can_id, data) = build_set_zero_frame(self.host_id, self.motor_id);
        self.send_frame(can_id, &data)?;
        std::thread::sleep(Duration::from_millis(50));
        Ok(())
    }

    /// Set the motor run mode.
    ///
    /// The motor should be disabled before changing run mode.
    pub fn set_run_mode(&mut self, mode: RunMode) -> Result<()> {
        let (can_id, data) = build_run_mode_frame(self.host_id, self.motor_id, mode);
        self.send_frame(can_id, &data)?;
        self.run_mode = mode;
        std::thread::sleep(Duration::from_millis(10));
        Ok(())
    }

    // =========================================================================
    // MIT mode control
    // =========================================================================

    /// Send a MIT-mode control command.
    ///
    /// Position, velocity, and torque use signed encoding centered at 0x7FFF.
    /// Kp and kd use unsigned encoding from 0 to scale.
    pub fn mit_control(
        &self,
        position: f64,
        velocity: f64,
        kp: f64,
        kd: f64,
        torque: f64,
    ) -> Result<MotorFeedback> {
        if !self.enabled {
            return Err(RobstrideError::NotEnabled {
                motor_id: self.motor_id,
            });
        }

        let (can_id, data) = build_mit_frame(
            self.host_id,
            self.motor_id,
            &self.scales,
            position,
            velocity,
            kp,
            kd,
            torque,
        );
        self.send_frame(can_id, &data)?;
        self.recv_status()
    }

    // =========================================================================
    // Position mode
    // =========================================================================

    /// Set target position (in position mode).
    pub fn set_position(&self, position: f32) -> Result<()> {
        let (can_id, data) = build_write_param_f32_frame(
            self.host_id,
            self.motor_id,
            ParamIndex::LocRef,
            position,
        );
        self.send_frame(can_id, &data)?;
        Ok(())
    }

    /// Set the position mode speed limit (rad/s).
    pub fn set_position_speed_limit(&self, speed: f32) -> Result<()> {
        let (can_id, data) = build_write_param_f32_frame(
            self.host_id,
            self.motor_id,
            ParamIndex::LimitSpd,
            speed,
        );
        self.send_frame(can_id, &data)?;
        Ok(())
    }

    /// Set the torque limit (Nm).
    pub fn set_torque_limit(&self, torque: f32) -> Result<()> {
        let (can_id, data) = build_write_param_f32_frame(
            self.host_id,
            self.motor_id,
            ParamIndex::LimitTorque,
            torque,
        );
        self.send_frame(can_id, &data)?;
        Ok(())
    }

    /// Set the current limit (A).
    pub fn set_current_limit(&self, current: f32) -> Result<()> {
        let (can_id, data) = build_write_param_f32_frame(
            self.host_id,
            self.motor_id,
            ParamIndex::LimitCur,
            current,
        );
        self.send_frame(can_id, &data)?;
        Ok(())
    }

    // =========================================================================
    // Velocity mode
    // =========================================================================

    /// Set target velocity (in velocity mode).
    pub fn set_velocity(&self, velocity: f32) -> Result<()> {
        let (can_id, data) = build_write_param_f32_frame(
            self.host_id,
            self.motor_id,
            ParamIndex::SpdRef,
            velocity,
        );
        self.send_frame(can_id, &data)?;
        Ok(())
    }

    // =========================================================================
    // Torque (current) mode
    // =========================================================================

    /// Set target torque / current (in torque mode).
    pub fn set_torque(&self, iq: f32) -> Result<()> {
        let (can_id, data) = build_write_param_f32_frame(
            self.host_id,
            self.motor_id,
            ParamIndex::IqRef,
            iq,
        );
        self.send_frame(can_id, &data)?;
        Ok(())
    }

    // =========================================================================
    // Parameter access
    // =========================================================================

    /// Read a parameter from the motor.
    pub fn read_param(&self, param: ParamIndex) -> Result<f32> {
        let (can_id, data) = build_read_param_frame(self.host_id, self.motor_id, param);
        self.send_frame(can_id, &data)?;

        // Wait for the read response (comm_type = ReadParameter = 17)
        let (_comm_type, _extra_data, _device_id, resp_data) = self.recv_frame()?;

        let (_idx, val) = parse_param_response(&resp_data).ok_or(
            RobstrideError::InvalidResponse {
                msg: "Failed to parse parameter response".into(),
            },
        )?;
        Ok(val)
    }

    /// Write a float parameter to the motor.
    pub fn write_param_f32(&self, param: ParamIndex, value: f32) -> Result<()> {
        let (can_id, data) =
            build_write_param_f32_frame(self.host_id, self.motor_id, param, value);
        self.send_frame(can_id, &data)?;
        std::thread::sleep(Duration::from_millis(5));
        Ok(())
    }

    // =========================================================================
    // Status reading
    // =========================================================================

    /// Read the current motor status using a zero MIT control command.
    pub fn read_status(&self) -> Result<MotorFeedback> {
        let (can_id, data) = build_mit_frame(
            self.host_id,
            self.motor_id,
            &self.scales,
            0.0, 0.0, 0.0, 0.0, 0.0,
        );
        self.send_frame(can_id, &data)?;
        self.recv_status()
    }

    /// Read the motor's mechanical position via parameter read.
    pub fn read_position(&self) -> Result<f32> {
        self.read_param(ParamIndex::MechPos)
    }

    /// Read the motor's mechanical velocity via parameter read.
    pub fn read_velocity(&self) -> Result<f32> {
        self.read_param(ParamIndex::MechVel)
    }

    /// Read the motor's filtered Iq (current) via parameter read.
    pub fn read_current(&self) -> Result<f32> {
        self.read_param(ParamIndex::IqFilt)
    }

    /// Read the motor's bus voltage.
    pub fn read_vbus(&self) -> Result<f32> {
        self.read_param(ParamIndex::Vbus)
    }

    // =========================================================================
    // Bus scanning
    // =========================================================================

    /// Scan the CAN bus for all responding motors.
    ///
    /// Sends a GET_DEVICE_ID (ping) to each motor ID in `id_range`
    /// and collects responses.
    pub fn scan_bus(
        interface: &str,
        host_id: u8,
        id_range: std::ops::RangeInclusive<u8>,
        timeout_per_id: Duration,
    ) -> Vec<(u8, Option<Vec<u8>>)> {
        let socket = match CanSocket::open(interface) {
            Ok(s) => s,
            Err(e) => {
                log::error!("Failed to open CAN socket '{}': {}", interface, e);
                return vec![];
            }
        };
        let _ = socket.set_read_timeout(timeout_per_id);

        let mut found = Vec::new();

        for motor_id in id_range {
            let (can_id, data) = build_ping_frame(host_id, motor_id);

            let ext_id = match ExtendedId::new(can_id) {
                Some(id) => id,
                None => continue,
            };
            let frame = match socketcan::CanFrame::new(ext_id, &data) {
                Some(f) => f,
                None => continue,
            };

            if socket.write_frame(&frame).is_err() {
                continue;
            }

            log::debug!("SCAN: probing motor_id={}", motor_id);

            // Try to read a response
            let start = Instant::now();
            while start.elapsed() < timeout_per_id {
                match socket.read_frame() {
                    Ok(resp) => {
                        if !resp.is_extended() {
                            continue;
                        }
                        let raw_id = match resp.id() {
                            Id::Standard(sid) => StandardId::as_raw(&sid) as u32,
                            Id::Extended(eid) => ExtendedId::as_raw(&eid),
                        };
                        let (ct, extra, dev_id) = parse_can_id(raw_id);

                        // In Robstride responses, the CAN ID layout is:
                        //   device_id field (bits 7-0)  = host_id (echo back)
                        //   extra_data field (bits 23-8) = motor_id | status_bits
                        // The motor_id is in the lower 8 bits of extra_data.
                        let resp_motor_id = (extra & 0xFF) as u8;

                        // Skip our own TX echo frames (gs_usb ECHO flag)
                        // Echo frames have the same CAN ID we sent
                        if dev_id == motor_id && ct == 0 {
                            // This is our own ping being echoed back, skip
                            continue;
                        }

                        // Check if this response is from the motor we pinged
                        if resp_motor_id == motor_id || dev_id == motor_id {
                            found.push((motor_id, Some(resp.data().to_vec())));
                            break;
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                    Err(_) => break,
                }
            }
        }

        found
    }

    /// Scan the CAN bus with progress callback.
    ///
    /// Calls `on_progress(current_index, total, motor_id)` for each probed ID.
    /// This allows the caller to update a UI progress bar during scanning.
    pub fn scan_bus_progressive<F>(
        interface: &str,
        host_id: u8,
        id_range: std::ops::RangeInclusive<u8>,
        timeout_per_id: Duration,
        mut on_progress: F,
    ) -> Vec<(u8, Option<Vec<u8>>)>
    where
        F: FnMut(usize, usize, u8),
    {
        let ids: Vec<u8> = id_range.collect();
        let total = ids.len();

        let socket = match CanSocket::open(interface) {
            Ok(s) => s,
            Err(e) => {
                log::error!("Failed to open CAN socket '{}': {}", interface, e);
                return vec![];
            }
        };
        let _ = socket.set_read_timeout(timeout_per_id);

        let mut found = Vec::new();

        for (idx, &motor_id) in ids.iter().enumerate() {
            on_progress(idx, total, motor_id);

            let (can_id, data) = build_ping_frame(host_id, motor_id);

            let ext_id = match ExtendedId::new(can_id) {
                Some(id) => id,
                None => continue,
            };
            let frame = match socketcan::CanFrame::new(ext_id, &data) {
                Some(f) => f,
                None => continue,
            };

            if socket.write_frame(&frame).is_err() {
                continue;
            }

            let start = Instant::now();
            while start.elapsed() < timeout_per_id {
                match socket.read_frame() {
                    Ok(resp) => {
                        if !resp.is_extended() {
                            continue;
                        }
                        let raw_id = match resp.id() {
                            Id::Standard(sid) => StandardId::as_raw(&sid) as u32,
                            Id::Extended(eid) => ExtendedId::as_raw(&eid),
                        };
                        let (ct, extra, dev_id) = parse_can_id(raw_id);
                        let resp_motor_id = (extra & 0xFF) as u8;
                        if dev_id == motor_id && ct == 0 {
                            continue;
                        }
                        if resp_motor_id == motor_id || dev_id == motor_id {
                            found.push((motor_id, Some(resp.data().to_vec())));
                            break;
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                    Err(_) => break,
                }
            }
        }

        on_progress(total, total, 0);
        found
    }

    /// Passively listen on the CAN bus and report all frames seen.
    pub fn dump_bus(
        interface: &str,
        duration: Duration,
    ) -> Vec<(u32, Vec<u8>)> {
        let socket = match CanSocket::open(interface) {
            Ok(s) => s,
            Err(e) => {
                log::error!("Failed to open CAN socket '{}': {}", interface, e);
                return vec![];
            }
        };
        let _ = socket.set_read_timeout(Duration::from_millis(100));

        let mut frames = Vec::new();
        let start = Instant::now();

        while start.elapsed() < duration {
            match socket.read_frame() {
                Ok(frame) => {
                    let raw_id = match frame.id() {
                        Id::Standard(sid) => StandardId::as_raw(&sid) as u32,
                        Id::Extended(eid) => ExtendedId::as_raw(&eid),
                    };
                    frames.push((raw_id, frame.data().to_vec()));
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(_) => break,
            }
        }

        frames
    }
}

/// Helper to reconstruct a raw CAN ID from parsed components.
fn build_can_id_raw(comm_type: u8, extra_data: u16, device_id: u8) -> u32 {
    ((comm_type as u32) << 24) | ((extra_data as u32) << 8) | (device_id as u32)
}

impl Drop for Motor {
    fn drop(&mut self) {
        if self.enabled {
            let _ = self.disable();
        }
    }
}
