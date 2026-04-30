//! CAN transport abstraction supporting both Classic CAN and CAN FD.
//!
//! The Robstride/DAMIAO protocols themselves are framing-agnostic: every
//! command is an 8-byte payload addressed by an 11-bit (DAMIAO) or 29-bit
//! (Robstride) CAN ID. Whether those bytes ride on a Classic CAN 2.0 frame or
//! a CAN FD frame is purely a transport decision. [`CanBus`] hides that choice
//! behind a uniform send/receive API so the rest of the crate (drivers,
//! high-level `Motor`, bilateral loop, TUI) never has to branch on it.
//!
//! When [`CanBus::Fd`] is selected, frames are sent as CAN FD with the **BRS**
//! (Bit Rate Switch) flag set, so the data phase runs at the interface's
//! configured `dbitrate`. This matches the usual Robstride CAN FD wiring; the
//! interface must therefore be brought up with an `fd on` / `dbitrate` config
//! (e.g. `ip link set canX type can bitrate 1000000 dbitrate 5000000 fd on`).

use std::io;
use std::time::Duration;

use socketcan::{
    CanAnyFrame, CanFdFrame, CanFdSocket, CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Id,
    Socket, StandardId,
};

/// A decoded receive frame, abstracted over the underlying socket kind.
///
/// Carries just the three things every caller in this crate needs from a
/// received frame: whether it used a 29-bit extended ID, the raw ID value,
/// and the payload bytes.
#[derive(Debug, Clone)]
pub struct RxFrame {
    /// True if the frame used a 29-bit extended identifier.
    pub extended: bool,
    /// Raw CAN identifier (11-bit value for standard, 29-bit for extended).
    pub raw_id: u32,
    /// Payload bytes (≤8 for Classic, ≤64 for CAN FD).
    pub data: Vec<u8>,
}

/// CAN transport: either a Classic CAN socket or a CAN FD socket.
///
/// Open one with [`CanBus::open`], passing `fd = true` to use CAN FD. All send
/// helpers build the appropriate frame kind automatically.
pub enum CanBus {
    /// Classic CAN 2.0 transport (≤8-byte payloads, no BRS).
    Classic(CanSocket),
    /// CAN FD transport (≤64-byte payloads, BRS enabled on TX).
    Fd(CanFdSocket),
}

impl CanBus {
    /// Open the named interface. `fd = true` opens a CAN FD socket; otherwise a
    /// Classic CAN socket.
    pub fn open(interface: &str, fd: bool) -> io::Result<Self> {
        if fd {
            Ok(CanBus::Fd(CanFdSocket::open(interface)?))
        } else {
            Ok(CanBus::Classic(CanSocket::open(interface)?))
        }
    }

    /// Whether this transport is CAN FD.
    pub fn is_fd(&self) -> bool {
        matches!(self, CanBus::Fd(_))
    }

    /// Set the blocking read timeout used by [`CanBus::read_frame`].
    pub fn set_read_timeout(&self, duration: Duration) -> io::Result<()> {
        match self {
            CanBus::Classic(s) => s.set_read_timeout(duration),
            CanBus::Fd(s) => s.set_read_timeout(duration),
        }
    }

    /// Set the blocking write timeout for the send helpers.
    ///
    /// Without this, a `write_*` call blocks indefinitely when the kernel TX
    /// queue backs up (e.g. frames aren't being ACKed because the bus is quiet
    /// or congested). With it set, a stalled send returns `WouldBlock`/`TimedOut`
    /// instead of hanging the calling thread — important for bus scans that
    /// must keep making progress across many IDs.
    pub fn set_write_timeout(&self, duration: Duration) -> io::Result<()> {
        match self {
            CanBus::Classic(s) => s.set_write_timeout(duration),
            CanBus::Fd(s) => s.set_write_timeout(duration),
        }
    }

    /// Send a frame addressed by a 29-bit extended ID (Robstride framing).
    pub fn write_extended(&self, can_id: u32, data: &[u8]) -> io::Result<()> {
        let id = ExtendedId::new(can_id).expect("Invalid extended CAN ID");
        self.write_id(Id::Extended(id), data)
    }

    /// Send a frame addressed by an 11-bit standard ID (DAMIAO framing).
    pub fn write_standard(&self, std_id: u16, data: &[u8]) -> io::Result<()> {
        let id = StandardId::new(std_id).expect("Invalid standard CAN ID");
        self.write_id(Id::Standard(id), data)
    }

    /// Build and write the correct frame kind for this transport.
    ///
    /// CAN FD frames are sent with BRS enabled so the data phase uses the
    /// interface's configured `dbitrate`.
    fn write_id(&self, id: Id, data: &[u8]) -> io::Result<()> {
        match self {
            CanBus::Classic(s) => {
                let frame = CanFrame::new(id, data).expect("Failed to create CAN frame");
                s.write_frame(&frame)
            }
            CanBus::Fd(s) => {
                let mut frame = CanFdFrame::new(id, data).expect("Failed to create CAN FD frame");
                frame.set_brs(true);
                s.write_frame(&frame)
            }
        }
    }

    /// Read the next frame, honouring the configured read timeout.
    ///
    /// Returns `WouldBlock` on timeout (same as the underlying socket), so
    /// callers can poll in a loop exactly as they did with the raw socket.
    pub fn read_frame(&self) -> io::Result<RxFrame> {
        match self {
            CanBus::Classic(s) => s.read_frame().map(|f| rx_from_classic(&f)),
            CanBus::Fd(s) => s.read_frame().map(|f| rx_from_any(&f)),
        }
    }

    /// Read the next frame with an explicit per-call timeout.
    pub fn read_frame_timeout(&self, timeout: Duration) -> io::Result<RxFrame> {
        match self {
            CanBus::Classic(s) => s.read_frame_timeout(timeout).map(|f| rx_from_classic(&f)),
            CanBus::Fd(s) => s.read_frame_timeout(timeout).map(|f| rx_from_any(&f)),
        }
    }
}

fn raw_id_of<F: EmbeddedFrame>(frame: &F) -> u32 {
    match frame.id() {
        Id::Standard(sid) => StandardId::as_raw(&sid) as u32,
        Id::Extended(eid) => ExtendedId::as_raw(&eid),
    }
}

fn rx_from_classic(frame: &CanFrame) -> RxFrame {
    RxFrame {
        extended: frame.is_extended(),
        raw_id: raw_id_of(frame),
        data: frame.data().to_vec(),
    }
}

fn rx_from_any(frame: &CanAnyFrame) -> RxFrame {
    RxFrame {
        extended: frame.is_extended(),
        raw_id: raw_id_of(frame),
        data: frame.data().to_vec(),
    }
}
