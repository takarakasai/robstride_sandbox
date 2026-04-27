//! Exclusive lock for a CAN interface name.
//!
//! Two processes that share a CAN interface cannot reliably co-control the
//! same motor: SocketCAN happily lets both open the bus, both write
//! frames, and both consume any motor response — so each process ends up
//! interpreting frames addressed to the *other* host as its own state
//! feedback. The result is wildly oscillatory bilateral control or
//! mysterious "ghost" position jumps.
//!
//! We guard against this with an advisory `flock(2)` on
//! `/tmp/robstride-<iface>.lock`. The lock is bound to the file descriptor
//! and released automatically when the process exits (graceful *or*
//! crash), so there is no stale-lock failure mode. We also write the
//! holder's PID into the file so the second process gets an actionable
//! error message instead of a cryptic "Resource temporarily unavailable".
//!
//! This is intentionally Linux/Unix-only — the rest of the crate is too.

use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::os::unix::io::AsRawFd;
use std::path::PathBuf;

use crate::error::{Result, RobstrideError};

/// RAII guard that holds an exclusive advisory lock on a CAN interface.
///
/// Drop the guard (or let it go out of scope) to release the lock and
/// remove the lock file. The lock is also released by the kernel if the
/// owning process dies before drop runs.
pub struct CanInterfaceLock {
    /// The locked file. The `flock(2)` lifetime is tied to this FD; the
    /// file is closed (and the lock dropped) when the guard is dropped.
    file: File,
    /// Path of the lock file, so `Drop` can `unlink` it. Best-effort —
    /// failure to remove is silent (the lock itself is already gone).
    path: PathBuf,
}

impl CanInterfaceLock {
    /// Lock `iface` exclusively. Fails immediately (no blocking) if another
    /// process already holds the lock; the error message includes the
    /// holder's PID so the user can find / stop it.
    pub fn acquire(iface: &str) -> Result<Self> {
        let path = lock_path(iface);

        // Open or create with rw permissions for us. World-readable so a
        // diagnostic `cat` from a normal user shows the holder's PID even
        // if the original launcher ran as a different user.
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .mode_if_unix(0o644)
            .open(&path)
            .map_err(|e| {
                RobstrideError::Other(format!(
                    "failed to open lock file {}: {}",
                    path.display(),
                    e
                ))
            })?;

        // Non-blocking exclusive lock. EWOULDBLOCK ⇒ someone else holds it.
        let fd = file.as_raw_fd();
        // SAFETY: `fd` is a valid file descriptor owned by `file` for the
        // duration of this call. flock with LOCK_EX|LOCK_NB has well-
        // defined behaviour on Linux.
        let rc = unsafe { libc::flock(fd, libc::LOCK_EX | libc::LOCK_NB) };
        if rc != 0 {
            let errno = std::io::Error::last_os_error();
            // Try to read the holder PID for a helpful message.
            let holder = read_pid(&path).unwrap_or_else(|| "<unknown>".to_string());
            return Err(RobstrideError::Other(format!(
                "CAN interface '{}' is already in use by PID {} \
                 (lock {}). Stop the other process or use a different \
                 interface. (errno: {})",
                iface,
                holder,
                path.display(),
                errno
            )));
        }

        // We hold the lock. Stamp the file with our PID so a later attempt
        // sees who to blame.
        let mut f2 = &file;
        // Truncate and write fresh content (previous holder may have left
        // a stale PID before crashing).
        if let Err(e) = ftruncate(fd, 0) {
            // Non-fatal — the lock itself is what matters.
            log::warn!("could not truncate lock file: {e}");
        }
        let pid = std::process::id();
        let _ = writeln!(f2, "{pid} {iface}");

        Ok(Self { file, path })
    }
}

impl Drop for CanInterfaceLock {
    fn drop(&mut self) {
        // Best-effort cleanup of the file. The flock is released
        // automatically when `self.file` is closed (after this Drop).
        let _ = std::fs::remove_file(&self.path);
        // Touch self.file so the compiler keeps it alive until after the
        // unlink succeeds (otherwise the field could be dropped first,
        // releasing the lock, and another process could grab+truncate
        // before our unlink runs — at worst that just deletes their
        // freshly-written PID stamp; no correctness issue, but a clearer
        // ordering for future readers).
        let _ = self.file.as_raw_fd();
    }
}

fn lock_path(iface: &str) -> PathBuf {
    // Sanitise interface name (no slashes etc.) — practical CAN names are
    // simple ("can0", "vcan1") but defensive coding is cheap here.
    let safe: String = iface
        .chars()
        .map(|c| if c.is_alphanumeric() || c == '-' || c == '_' { c } else { '_' })
        .collect();
    PathBuf::from(format!("/tmp/robstride-{safe}.lock"))
}

fn read_pid(path: &std::path::Path) -> Option<String> {
    let mut buf = String::new();
    File::open(path).ok()?.read_to_string(&mut buf).ok()?;
    // First whitespace-separated token is the PID.
    buf.split_whitespace().next().map(|s| s.to_string())
}

fn ftruncate(fd: i32, len: i64) -> std::io::Result<()> {
    // SAFETY: ftruncate on a valid fd is well-defined.
    let rc = unsafe { libc::ftruncate(fd, len) };
    if rc != 0 {
        Err(std::io::Error::last_os_error())
    } else {
        Ok(())
    }
}

// Helper to set mode at open(2) time on Unix without a cfg block at the
// call site. On any other platform `mode` is a no-op (we don't actually
// target other platforms, but this keeps the call site readable).
trait OpenOptionsExt {
    fn mode_if_unix(&mut self, mode: u32) -> &mut Self;
}

impl OpenOptionsExt for OpenOptions {
    #[cfg(unix)]
    fn mode_if_unix(&mut self, mode: u32) -> &mut Self {
        use std::os::unix::fs::OpenOptionsExt as _;
        self.mode(mode)
    }
    #[cfg(not(unix))]
    fn mode_if_unix(&mut self, _mode: u32) -> &mut Self {
        self
    }
}
