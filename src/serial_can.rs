//! Direct serial port communication with USB-CAN adapters.
//!
//! Bypasses SocketCAN/slcand and communicates directly via `/dev/ttyUSB0`.
//! Supports SLCAN (ASCII) protocol and raw byte sniffing for unknown adapters.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use crate::error::{Result, RobstrideError};
use crate::protocol::*;

/// SLCAN speed code for the `Sx` command.
fn bitrate_to_slcan_code(bitrate: u32) -> Option<u8> {
    match bitrate {
        10_000 => Some(0),
        20_000 => Some(1),
        50_000 => Some(2),
        100_000 => Some(3),
        125_000 => Some(4),
        250_000 => Some(5),
        500_000 => Some(6),
        800_000 => Some(7),
        1_000_000 => Some(8),
        _ => None,
    }
}

/// Direct serial CAN transport.
///
/// Talks to a USB-CAN adapter via serial port using SLCAN ASCII protocol.
pub struct SerialCan {
    port: Box<dyn serialport::SerialPort>,
}

impl SerialCan {
    /// Open a serial port and initialize SLCAN.
    ///
    /// # Arguments
    /// * `device` - Serial device path (e.g., "/dev/ttyUSB0")
    /// * `baud` - Serial baud rate (typically 115200 or 921600)
    /// * `can_bitrate` - CAN bus bitrate (e.g., 1_000_000)
    pub fn open(device: &str, baud: u32, can_bitrate: u32) -> Result<Self> {
        let port = serialport::new(device, baud)
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| RobstrideError::SerialPort(e.to_string()))?;

        let mut sc = SerialCan {
            port,
        };

        // Initialize SLCAN: Close, set speed, open
        sc.slcan_cmd("C")?; // Close channel (reset)
        std::thread::sleep(Duration::from_millis(100));

        let speed_code = bitrate_to_slcan_code(can_bitrate).ok_or_else(|| {
            RobstrideError::SerialPort(format!("Unsupported CAN bitrate: {}", can_bitrate))
        })?;
        sc.slcan_cmd(&format!("S{}", speed_code))?; // Set CAN speed
        std::thread::sleep(Duration::from_millis(50));

        sc.slcan_cmd("O")?; // Open channel
        std::thread::sleep(Duration::from_millis(50));

        // Drain any pending data
        sc.drain()?;

        log::info!(
            "Serial CAN opened: {} @ {}baud, CAN {}bps",
            device,
            baud,
            can_bitrate
        );

        Ok(sc)
    }

    /// Send a raw SLCAN command (without CR terminator - we add it).
    fn slcan_cmd(&mut self, cmd: &str) -> Result<()> {
        let msg = format!("{}\r", cmd);
        self.port
            .write_all(msg.as_bytes())
            .map_err(|e| RobstrideError::SerialPort(e.to_string()))?;
        self.port
            .flush()
            .map_err(|e| RobstrideError::SerialPort(e.to_string()))?;
        log::debug!("SLCAN TX cmd: {:?}", cmd);
        std::thread::sleep(Duration::from_millis(10));
        Ok(())
    }

    /// Drain any pending bytes from the serial port.
    fn drain(&mut self) -> Result<()> {
        let mut buf = [0u8; 256];
        loop {
            match self.port.read(&mut buf) {
                Ok(0) => break,
                Ok(_) => continue,
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => break,
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(RobstrideError::SerialPort(e.to_string())),
            }
        }
        Ok(())
    }

    /// Send a CAN frame (extended 29-bit ID).
    ///
    /// SLCAN format for extended frame:
    ///   `T<ID:8hex><DLC:1><DATA:2*DLC>\r`
    pub fn send_frame(&mut self, can_id: u32, data: &[u8]) -> Result<()> {
        let dlc = data.len().min(8);
        let mut msg = format!("T{:08X}{}", can_id, dlc);
        for &b in &data[..dlc] {
            msg.push_str(&format!("{:02X}", b));
        }
        msg.push('\r');

        self.port
            .write_all(msg.as_bytes())
            .map_err(|e| RobstrideError::SerialPort(e.to_string()))?;
        self.port
            .flush()
            .map_err(|e| RobstrideError::SerialPort(e.to_string()))?;

        log::debug!("SLCAN TX: {}", msg.trim());
        Ok(())
    }

    /// Try to receive a CAN frame with timeout.
    ///
    /// SLCAN response format for extended frame:
    ///   `T<ID:8hex><DLC:1><DATA:2*DLC>\r`
    /// or `t<ID:3hex><DLC:1><DATA:2*DLC>\r` for standard frames.
    pub fn recv_frame(&mut self, timeout: Duration) -> Result<Option<(u32, Vec<u8>)>> {
        let start = Instant::now();
        let mut line_buf = Vec::new();

        while start.elapsed() < timeout {
            let mut byte = [0u8; 1];
            match self.port.read(&mut byte) {
                Ok(1) => {
                    if byte[0] == b'\r' || byte[0] == b'\n' || byte[0] == 7 {
                        // End of SLCAN message (or bell = error)
                        if byte[0] == 7 {
                            log::debug!("SLCAN: received BELL (error/nack)");
                            line_buf.clear();
                            continue;
                        }
                        if !line_buf.is_empty() {
                            let line = String::from_utf8_lossy(&line_buf).to_string();
                            log::debug!("SLCAN RX: {:?}", line);
                            let result = Self::parse_slcan_frame(&line);
                            line_buf.clear();
                            if result.is_some() {
                                return Ok(result);
                            }
                        }
                    } else {
                        line_buf.push(byte[0]);
                    }
                }
                Ok(_) => {}
                Err(ref e)
                    if e.kind() == std::io::ErrorKind::TimedOut
                        || e.kind() == std::io::ErrorKind::WouldBlock =>
                {
                    continue;
                }
                Err(e) => return Err(RobstrideError::SerialPort(e.to_string())),
            }
        }

        Ok(None)
    }

    /// Parse an SLCAN ASCII line into (CAN ID, data).
    fn parse_slcan_frame(line: &str) -> Option<(u32, Vec<u8>)> {
        let bytes = line.as_bytes();
        if bytes.is_empty() {
            return None;
        }

        match bytes[0] {
            b'T' if bytes.len() >= 10 => {
                // Extended frame: T<8hex_id><1dlc><data_hex>
                let id_str = &line[1..9];
                let can_id = u32::from_str_radix(id_str, 16).ok()?;
                let dlc = (bytes[9] as char).to_digit(10)? as usize;
                let expected_len = 10 + dlc * 2;
                if bytes.len() < expected_len {
                    return None;
                }
                let mut data = Vec::with_capacity(dlc);
                for i in 0..dlc {
                    let hex = &line[10 + i * 2..12 + i * 2];
                    data.push(u8::from_str_radix(hex, 16).ok()?);
                }
                Some((can_id, data))
            }
            b't' if bytes.len() >= 5 => {
                // Standard frame: t<3hex_id><1dlc><data_hex>
                let id_str = &line[1..4];
                let can_id = u32::from_str_radix(id_str, 16).ok()?;
                let dlc = (bytes[4] as char).to_digit(10)? as usize;
                let expected_len = 5 + dlc * 2;
                if bytes.len() < expected_len {
                    return None;
                }
                let mut data = Vec::with_capacity(dlc);
                for i in 0..dlc {
                    let hex = &line[5 + i * 2..7 + i * 2];
                    data.push(u8::from_str_radix(hex, 16).ok()?);
                }
                Some((can_id, data))
            }
            b'z' | b'Z' => {
                // Transmit OK confirmation
                log::debug!("SLCAN: TX confirmed");
                None
            }
            _ => {
                log::debug!("SLCAN: unknown response: {:?}", line);
                None
            }
        }
    }

    /// Send a frame and wait for a response.
    pub fn send_and_recv(
        &mut self,
        can_id: u32,
        data: &[u8],
        timeout: Duration,
    ) -> Result<Option<(u32, Vec<u8>)>> {
        self.send_frame(can_id, data)?;
        self.recv_frame(timeout)
    }

    /// Close the SLCAN channel.
    pub fn close(&mut self) -> Result<()> {
        self.slcan_cmd("C")?;
        log::info!("Serial CAN closed");
        Ok(())
    }

    /// Scan for motors on the bus.
    ///
    /// For each ID in range, sends GetDeviceId + Enable/Disable probe.
    pub fn scan(
        &mut self,
        host_id: u8,
        id_range: std::ops::RangeInclusive<u8>,
        timeout_per_id: Duration,
    ) -> Vec<(u8, Option<MotorFeedback>)> {
        let mut found = Vec::new();
        // Use a default scale for decoding scan responses
        let scales = MitScales::for_model(MotorModel::Rs05);

        for motor_id in id_range {
            // Probe with GetDeviceId (ping)
            let (can_id, ping_data) = build_ping_frame(host_id, motor_id);
            if self.send_frame(can_id, &ping_data).is_err() {
                continue;
            }
            log::debug!("SERIAL SCAN: probing motor_id={} (GetDeviceId)", motor_id);

            if let Ok(Some((resp_id, resp_data))) = self.recv_frame(timeout_per_id) {
                let (_ct, _extra, dev_id) = parse_can_id(resp_id);
                if dev_id == motor_id || (resp_id & 0xFF) == motor_id as u32 {
                    let feedback = parse_status_frame(resp_id, &resp_data, &scales);
                    found.push((motor_id, feedback));
                    continue;
                }
            }

            // Fallback: try Enable
            let (en_id, en_data) = build_enable_frame(host_id, motor_id);
            if self.send_frame(en_id, &en_data).is_err() {
                continue;
            }

            if let Ok(Some((resp_id, resp_data))) = self.recv_frame(timeout_per_id) {
                let (_ct, _extra, dev_id) = parse_can_id(resp_id);
                if dev_id == motor_id || (resp_id & 0xFF) == motor_id as u32 {
                    let feedback = parse_status_frame(resp_id, &resp_data, &scales);
                    found.push((motor_id, feedback));

                    // Disable immediately
                    let (dis_id, dis_data) = build_disable_frame(host_id, motor_id);
                    let _ = self.send_frame(dis_id, &dis_data);
                    let _ = self.recv_frame(Duration::from_millis(20));
                }
            }
        }

        found
    }

    /// Dump all raw bytes from the serial port for analysis.
    ///
    /// When the adapter protocol is unknown, this helps identify
    /// what data format is being used.
    pub fn dump_raw(device: &str, baud: u32, duration: Duration) -> Vec<u8> {
        let port = match serialport::new(device, baud)
            .timeout(Duration::from_millis(100))
            .open()
        {
            Ok(p) => p,
            Err(e) => {
                log::error!("Failed to open serial port '{}': {}", device, e);
                return vec![];
            }
        };

        let mut reader = std::io::BufReader::new(port);
        let mut all_bytes = Vec::new();
        let mut buf = [0u8; 256];
        let start = Instant::now();

        while start.elapsed() < duration {
            match reader.read(&mut buf) {
                Ok(0) => continue,
                Ok(n) => {
                    all_bytes.extend_from_slice(&buf[..n]);
                }
                Err(ref e)
                    if e.kind() == std::io::ErrorKind::TimedOut
                        || e.kind() == std::io::ErrorKind::WouldBlock =>
                {
                    continue;
                }
                Err(_) => break,
            }
        }

        all_bytes
    }

    /// Dump raw bytes and attempt to interpret as SLCAN ASCII,
    /// printing both hex and ASCII views.
    pub fn dump_raw_pretty(device: &str, baud: u32, duration: Duration) {
        println!(
            "Raw serial dump from {} @ {}baud for {:.1}s...",
            device,
            baud,
            duration.as_secs_f32()
        );
        println!("(First trying to send SLCAN version query)\n");

        let mut port = match serialport::new(device, baud)
            .timeout(Duration::from_millis(200))
            .open()
        {
            Ok(p) => p,
            Err(e) => {
                println!("Failed to open serial port '{}': {}", device, e);
                return;
            }
        };

        // Send version query
        let _ = port.write_all(b"V\r");
        let _ = port.flush();
        std::thread::sleep(Duration::from_millis(200));

        // Send serial number query
        let _ = port.write_all(b"N\r");
        let _ = port.flush();
        std::thread::sleep(Duration::from_millis(200));

        let mut all_bytes = Vec::new();
        let mut buf = [0u8; 256];
        let start = Instant::now();

        while start.elapsed() < duration {
            match port.read(&mut buf) {
                Ok(0) => continue,
                Ok(n) => {
                    all_bytes.extend_from_slice(&buf[..n]);
                    // Print live
                    let hex: Vec<String> = buf[..n].iter().map(|b| format!("{:02X}", b)).collect();
                    let ascii: String = buf[..n]
                        .iter()
                        .map(|&b| if b.is_ascii_graphic() || b == b' ' { b as char } else { '.' })
                        .collect();
                    println!("  [{}]  |{}|", hex.join(" "), ascii);
                }
                Err(ref e)
                    if e.kind() == std::io::ErrorKind::TimedOut
                        || e.kind() == std::io::ErrorKind::WouldBlock =>
                {
                    continue;
                }
                Err(e) => {
                    println!("Read error: {}", e);
                    break;
                }
            }
        }

        if all_bytes.is_empty() {
            println!("\n  No data received from serial port.");
            println!("\n  Possible issues:");
            println!("    1. Adapter is not SLCAN compatible");
            println!("    2. Wrong baud rate (try 921600, 2000000, 460800)");
            println!("    3. Adapter needs a different protocol");
        } else {
            println!(
                "\n  Total: {} bytes received",
                all_bytes.len()
            );

            // Try to detect if it looks like SLCAN
            let has_ascii = all_bytes.iter().all(|&b| b.is_ascii() || b == b'\r' || b == b'\n');
            if has_ascii {
                println!("  Format: Looks like ASCII (possibly SLCAN compatible)");
                let text = String::from_utf8_lossy(&all_bytes);
                println!("  Content: {:?}", text);
            } else {
                println!("  Format: Binary data (NOT SLCAN - custom protocol)");
                println!("  Use this hex dump to identify the adapter's protocol.");
            }
        }
    }
}

impl Drop for SerialCan {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_extended_frame() {
        // T030001FD80000000000000000
        let line = "T030001FD80000000000000000";
        let result = SerialCan::parse_slcan_frame(line);
        assert!(result.is_some());
        let (id, data) = result.unwrap();
        assert_eq!(id, 0x030001FD);
        assert_eq!(data.len(), 8);
        assert_eq!(data, vec![0, 0, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn test_parse_standard_frame() {
        let line = "t0014DEADBEEF";
        let result = SerialCan::parse_slcan_frame(line);
        assert!(result.is_some());
        let (id, data) = result.unwrap();
        assert_eq!(id, 0x001);
        assert_eq!(data, vec![0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn test_bitrate_codes() {
        assert_eq!(bitrate_to_slcan_code(1_000_000), Some(8));
        assert_eq!(bitrate_to_slcan_code(500_000), Some(6));
        assert_eq!(bitrate_to_slcan_code(999), None);
    }
}
