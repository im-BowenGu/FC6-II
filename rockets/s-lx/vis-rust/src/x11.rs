//! Minimal X11 client: raw wire protocol over a Unix socket, no crates.
//!
//! Only the handful of requests this visualiser needs are implemented:
//! setup, CreateWindow, MapWindow, CreateGC, ChangeProperty, PutImage and
//! event draining. The frame is rasterised on the CPU and uploaded with
//! ZPixmap PutImage calls, split to respect the server's request limit.

use std::env;
use std::fs;
use std::io::{self, Read, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::time::Duration;

const XPAD: usize = 4;

fn w8(b: &mut Vec<u8>, v: u8) {
    b.push(v);
}
fn w16(b: &mut Vec<u8>, v: u16) {
    b.extend_from_slice(&v.to_le_bytes());
}
fn w32(b: &mut Vec<u8>, v: u32) {
    b.extend_from_slice(&v.to_le_bytes());
}
fn wi16(b: &mut Vec<u8>, v: i16) {
    b.extend_from_slice(&v.to_le_bytes());
}
fn pad4(b: &mut Vec<u8>) {
    while b.len() % XPAD != 0 {
        b.push(0);
    }
}
/// X11 request `length` is 32-bit words INCLUDING the 4-byte request header.
/// Every request builder below must use this helper, not words-after-header.
fn words(body_bytes: usize) -> u16 {
    ((4 + body_bytes) / 4) as u16
}

fn hex(b: &[u8]) -> String {
    let mut s = String::with_capacity(b.len() * 2);
    for x in b {
        s.push_str(&format!("{x:02x}"));
    }
    s
}

#[derive(Debug)]
#[allow(dead_code)] // masks are parsed for ABI validation; PutImage uses 0x00RRGGBB
pub struct X11 {
    stream: UnixStream,
    pending: Vec<u8>,
    max_request: usize,
    base: u32,
    id_mask: u32,
    next_id: u32,
    pub root: u32,
    pub white: u32,
    pub black: u32,
    pub visual: u32,
    pub depth: u8,
    pub red_mask: u32,
    pub green_mask: u32,
    pub blue_mask: u32,
    pub image_lsb: bool,
}

fn auth_cookie(display_num: u32) -> Option<(String, Vec<u8>)> {
    let path = env::var_os("XAUTHORITY")
        .map(PathBuf::from)
        .or_else(|| env::var_os("HOME").map(|h| PathBuf::from(h).join(".Xauthority")))?;
    let data = fs::read(path).ok()?;
    let mut cur = 0usize;
    while cur + 12 <= data.len() {
        let family = u16::from_le_bytes([data[cur], data[cur + 1]]);
        let alen = u16::from_le_bytes([data[cur + 2], data[cur + 3]]) as usize;
        let nlen = u16::from_le_bytes([data[cur + 4], data[cur + 5]]) as usize;
        let dlen = u16::from_le_bytes([data[cur + 8], data[cur + 9]]) as usize;
        let mut o = cur + 12;
        if o + alen + nlen + dlen > data.len() {
            break;
        }
        o += alen;
        let name = &data[o..o + nlen];
        o += nlen;
        let cookie = data[o..o + dlen].to_vec();
        let wanted = display_num.to_string();
        if family == 256 && name == wanted.as_bytes() {
            return Some(("MIT-MAGIC-COOKIE-1".to_string(), cookie));
        }
        cur = o;
    }
    None
}

impl X11 {
    pub fn connect() -> Result<Self, String> {
        let display = env::var("DISPLAY").unwrap_or_else(|_| ":0".to_string());
        let (host, num, screen) = parse_display(&display)?;
        if !host.is_empty() && host != "unix" {
            return Err(format!("only local X sockets are supported (DISPLAY={display})"));
        }
        let socket = format!("/tmp/.X11-unix/X{num}");
        let mut stream = UnixStream::connect(&socket)
            .map_err(|e| format!("connect {socket}: {e}"))?;
        stream.set_read_timeout(Some(Duration::from_secs(10))).ok();
        stream.set_write_timeout(Some(Duration::from_secs(10))).ok();

        let mut req = vec![0u8; 12];
        req[0] = b'l'; // little-endian wire format
        req[2] = 11; // protocol major
        let (name, cookie) = auth_cookie(num).unwrap_or_default();
        req[6..8].copy_from_slice(&(name.len() as u16).to_le_bytes());
        req[8..10].copy_from_slice(&(cookie.len() as u16).to_le_bytes());
        req.extend_from_slice(name.as_bytes());
        req.extend_from_slice(&cookie);
        pad4(&mut req);
        stream.write_all(&req).map_err(|e| format!("X setup write: {e}"))?;

        let mut head = [0u8; 8];
        stream.read_exact(&mut head).map_err(|e| format!("X setup read: {e}"))?;
        if head[0] != 1 {
            let n = u16::from_le_bytes([head[6], head[7]]) as usize * 4;
            let mut reason = vec![0u8; n];
            stream.read_exact(&mut reason).ok();
            return Err(format!(
                "X setup rejected (status {}, protocol {}.{}): {}",
                head[0], head[2], head[3],
                String::from_utf8_lossy(&reason[..reason.len().min(120)])
            ));
        }
        let nwords = u16::from_le_bytes([head[6], head[7]]) as usize;
        let mut body = vec![0u8; nwords * 4];
        stream.read_exact(&mut body).map_err(|e| format!("X setup body: {e}"))?;

        let u32at = |b: &[u8], o: usize| -> u32 {
            u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
        };
        let u16at = |b: &[u8], o: usize| -> u16 {
            u16::from_le_bytes([b[o], b[o + 1]])
        };

        let base = u32at(&body, 4);
        let id_mask = u32at(&body, 8);
        let vlen = u16at(&body, 16) as usize;
        let maxreq = u16at(&body, 18) as usize * 4;
        let nscreen = body[20] as usize;
        let npixfmt = body[21] as usize;
        let image_lsb = body[22] == 0;
        if screen >= nscreen {
            return Err(format!("DISPLAY screen {screen} not present (server has {nscreen})"));
        }
        let mut off = 32 + ((vlen + 3) / 4) * 4;
        for _ in 0..npixfmt {
            if off + 8 > body.len() {
                return Err("truncated pixmap formats".into());
            }
            off += 8;
        }
        let mut root = 0u32;
        let mut white = 1u32;
        let mut black = 0u32;
        let mut root_visual = 0u32;
        let mut rootdepth = 24u8;
        for i in 0..nscreen {
            if off + 40 > body.len() {
                return Err("truncated screens".into());
            }
            if i == screen {
                root = u32at(&body, off);
                white = u32at(&body, off + 8);
                black = u32at(&body, off + 12);
                root_visual = u32at(&body, off + 32);
                rootdepth = body[off + 38];
            }
            let ndepths = body[off + 39] as usize;
            off += 40;
            for _ in 0..ndepths {
                if off + 8 > body.len() {
                    return Err("truncated depth header".into());
                }
                let depth = body[off];
                let nvis = u16at(&body, off + 2) as usize;
                off += 8;
                if i == screen && depth == rootdepth && root_visual == 0 {
                    if off + 24 <= body.len() {
                        root_visual = u32at(&body, off);
                    }
                }
                off += 24 * nvis;
            }
        }

        // Resolve the root visual's RGB masks. Xwayland advertises a TrueColor
        // 24-bit visual whose pixel value is 0x00RRGGBB.
        let mut visual = root_visual;
        let mut rm = 0x00ff_0000u32;
        let mut gm = 0x0000_ff00u32;
        let mut bm = 0x0000_00ffu32;
        off = 32 + ((vlen + 3) / 4) * 4 + npixfmt * 8;
        for i in 0..nscreen {
            if off + 40 > body.len() {
                break;
            }
            let ndepths = body[off + 39] as usize;
            off += 40;
            let mut found = false;
            for _ in 0..ndepths {
                if off + 8 > body.len() {
                    break;
                }
                let depth = body[off];
                let nvis = u16at(&body, off + 2) as usize;
                off += 8;
                for _ in 0..nvis {
                    if off + 24 > body.len() {
                        break;
                    }
                    let vid = u32at(&body, off);
                    let class = body[off + 4];
                    if i == screen && vid == root_visual && class == 4 {
                        rm = u32at(&body, off + 8);
                        gm = u32at(&body, off + 12);
                        bm = u32at(&body, off + 16);
                        found = true;
                    }
                    if !found && i == screen && class == 4 && depth == 24 {
                        visual = vid;
                        rm = u32at(&body, off + 8);
                        gm = u32at(&body, off + 12);
                        bm = u32at(&body, off + 16);
                        found = true;
                    }
                    off += 24;
                }
            }
        }
        if maxreq < 4096 {
            return Err("server maximum request length is implausibly small".into());
        }

        Ok(Self {
            stream,
            pending: Vec::new(),
            max_request: maxreq,
            base,
            id_mask,
            next_id: 0x1000,
            root,
            white,
            black,
            visual,
            depth: rootdepth,
            red_mask: rm,
            green_mask: gm,
            blue_mask: bm,
            image_lsb,
        })
    }

    pub fn new_id(&mut self) -> u32 {
        let id = self.base | (self.next_id & self.id_mask);
        self.next_id = self.next_id.wrapping_add(0x1000);
        if id == 0 {
            self.base | 1
        } else {
            id
        }
    }

    fn send(&mut self, b: &[u8]) -> Result<(), String> {
        // PutImage is sent 15x per frame; logging it would drown out setup
        // diagnostics, so only trace the small control requests.
        if env::var_os("SLX_X_DEBUG").is_some() && b.first().copied() != Some(72) {
            let pre = if b.len() >= 4 { hex(&b[..4]) } else { hex(b) };
            eprintln!("XREQ opcode={} bytes={} head={}", b.first().copied().unwrap_or(255), b.len(), pre);
        }
        self.stream.write_all(b).map_err(|e| format!("X write: {e}"))
    }

    pub fn create_window(&mut self, w: u16, h: u16) -> Result<u32, String> {
        let id = self.new_id();
        // xCreateWindowReq is 32 bytes: 4-byte header + 28-byte body.
        // The two extra background/pixel words a naive port might add would
        // make the server reject the request with BadLength (16).
        let mut body = Vec::with_capacity(28);
        w32(&mut body, id);
        w32(&mut body, self.root);
        wi16(&mut body, 60);
        wi16(&mut body, 60);
        w16(&mut body, w);
        w16(&mut body, h);
        w16(&mut body, 0); // border width
        w16(&mut body, 1); // InputOutput
        w32(&mut body, self.visual);
        w32(&mut body, 0); // no attributes; events are selected separately
        let mut b = Vec::with_capacity(32);
        w8(&mut b, 1); // CreateWindow opcode (byte 0), depth follows in byte 1
        w8(&mut b, self.depth);
        w16(&mut b, words(body.len()));
        b.extend_from_slice(&body);
        if env::var_os("SLX_X_DEBUG").is_some() {
            eprintln!("CREATE_WINDOW root=0x{:x} visual=0x{:x} depth={} len={} bytes={}",
                      self.root, self.visual, self.depth, b.len(), hex(&b));
        }
        self.send(&b)?;
        Ok(id)
    }

    /// Select key/expose/structure events with ChangeWindowAttributes.
    /// CWEventMask is bit 11 and takes exactly one CARD32 value word.
    pub fn select_events(&mut self, win: u32, event_mask: u32) -> Result<(), String> {
        let mut body = Vec::with_capacity(12);
        w32(&mut body, win);
        w32(&mut body, 1 << 11); // CWEventMask
        w32(&mut body, event_mask);
        let mut b = Vec::with_capacity(16);
        w8(&mut b, 2); // ChangeWindowAttributes
        w8(&mut b, 0);
        w16(&mut b, words(body.len()));
        b.extend_from_slice(&body);
        self.send(&b)
    }

    pub fn map_window(&mut self, id: u32) -> Result<(), String> {
        let mut b = Vec::with_capacity(8);
        w8(&mut b, 8); // MapWindow
        w8(&mut b, 0);
        w16(&mut b, words(4)); // xResourceReq: 8 bytes total
        w32(&mut b, id);
        self.send(&b)
    }

    pub fn create_gc(&mut self, drawable: u32) -> Result<u32, String> {
        let id = self.new_id();
        let mut body = Vec::with_capacity(20);
        w32(&mut body, id);
        w32(&mut body, drawable);
        w32(&mut body, (1 << 2) | (1 << 3)); // GCForeground | GCBackground
        w32(&mut body, self.white);
        w32(&mut body, self.black);
        let mut b = Vec::with_capacity(24);
        w8(&mut b, 55); // CreateGC
        w8(&mut b, 0);
        w16(&mut b, words(body.len()));
        b.extend_from_slice(&body);
        self.send(&b)?;
        Ok(id)
    }

    /// Set the legacy WM_NAME property so Hyprland shows a useful title.
    pub fn set_title(&mut self, win: u32, title: &str) -> Result<(), String> {
        // For format=8, nUnits counts 8-bit units (bytes), NOT words. The data
        // itself is padded to a 4-byte boundary in the request body.
        let n_units = title.len() as u32;
        let mut data = title.as_bytes().to_vec();
        pad4(&mut data);
        let mut body = Vec::with_capacity(16 + data.len());
        w32(&mut body, win);
        w32(&mut body, 39); // WM_NAME
        w32(&mut body, 31); // STRING
        w8(&mut body, 8); // format
        w8(&mut body, 0);
        w16(&mut body, 0); // 3 pad bytes total
        w32(&mut body, n_units); // nUnits for format=8 is the byte count
        body.extend_from_slice(&data);
        let mut b = Vec::with_capacity(20 + data.len());
        w8(&mut b, 18); // ChangeProperty
        w8(&mut b, 0); // ReplaceMode
        w16(&mut b, words(body.len()));
        b.extend_from_slice(&body);
        self.send(&b)
    }

    /// Upload a 0x00RRGGBB framebuffer as a ZPixmap, split into row chunks.
    pub fn put_image(&mut self, win: u32, gc: u32, w: usize, h: usize, rgba: &[u32]) -> Result<(), String> {
        let bytes_per_row = w * 4;
        let overhead = 24;
        let max_rows = (self.max_request.saturating_sub(overhead) / bytes_per_row).max(1);
        let max_rows = max_rows.min(64);
        let mut y = 0usize;
        while y < h {
            let rows = (h - y).min(max_rows);
            let data_len = rows * bytes_per_row;
            let mut b = Vec::with_capacity(overhead + data_len);
            w8(&mut b, 72); // PutImage
            w8(&mut b, 2); // ZPixmap
            // 20 bytes of fixed fields after the 4-byte header, then raster.
            w16(&mut b, words(20 + data_len));
            w32(&mut b, win);
            w32(&mut b, gc);
            w16(&mut b, w as u16);
            w16(&mut b, rows as u16);
            wi16(&mut b, 0);
            wi16(&mut b, y as i16);
            w8(&mut b, 0); // left_pad
            w8(&mut b, self.depth);
            w16(&mut b, 0);
            for px in &rgba[y * w..(y + rows) * w] {
                if self.image_lsb {
                    b.extend_from_slice(&[
                        (*px & 0xff) as u8,
                        ((*px >> 8) & 0xff) as u8,
                        ((*px >> 16) & 0xff) as u8,
                        0,
                    ]);
                } else {
                    b.extend_from_slice(&[
                        0,
                        ((*px >> 16) & 0xff) as u8,
                        ((*px >> 8) & 0xff) as u8,
                        (*px & 0xff) as u8,
                    ]);
                }
            }
            self.send(&b)?;
            y += rows;
        }
        Ok(())
    }

    /// Force delivery of any pending X errors with a round-trip GetInputFocus.
    /// Without this, CreateWindow/PutImage failures are asynchronous and would
    /// otherwise be silently swallowed by the event drain in `poll_key`.
    pub fn check_errors(&mut self, label: &str) -> Result<(), String> {
        let mut req = [0u8; 4];
        req[0] = 43; // GetInputFocus (no arguments, 4-byte request)
        req[2..4].copy_from_slice(&1u16.to_le_bytes());
        self.send(&req)?;
        loop {
            let mut rec = [0u8; 32];
            self.stream
                .read_exact(&mut rec)
                .map_err(|e| format!("{label}: X sync read: {e}"))?;
            if rec[0] == 0 {
                let resource = u32::from_le_bytes([rec[4], rec[5], rec[6], rec[7]]);
                return Err(format!(
                    "{label}: X error code {} seq {} resource 0x{:08x} minor {} major {} raw {}",
                    rec[1],
                    u16::from_le_bytes([rec[2], rec[3]]),
                    resource,
                    u16::from_le_bytes([rec[8], rec[9]]),
                    rec[10],
                    hex(&rec)
                ));
            }
            if rec[0] == 1 {
                return Ok(());
            }
        }
    }

    /// Drain pending events; returns true when a key was pressed (quit hint).
    pub fn poll_key(&mut self) -> bool {
        self.stream.set_nonblocking(true).ok();
        let mut buf = [0u8; 4096];
        let mut quit = false;
        loop {
            match self.stream.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => self.pending.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
        self.stream.set_nonblocking(false).ok();
        while self.pending.len() >= 32 {
            let ev: [u8; 32] = self.pending[..32].try_into().unwrap();
            self.pending.drain(..32);
            if ev[0] == 2 || ev[0] == 3 {
                if env::var_os("SLX_X_DEBUG").is_some() {
                    eprintln!("POLL_KEY quit on event type {} raw {}", ev[0], hex(&ev));
                }
                quit = true;
            }
        }
        quit
    }
}

fn parse_display(s: &str) -> Result<(String, u32, usize), String> {
    let s = s.trim();
    let (host_num, screen) = match s.rsplit_once('.') {
        Some((a, b)) if b.chars().all(|c| c.is_ascii_digit()) => (a, b.parse().unwrap_or(0)),
        _ => (s, 0),
    };
    let (host, num) = match host_num.rsplit_once(':') {
        Some((h, n)) => (h.to_string(), n.parse::<u32>().map_err(|_| format!("bad display number in {s}"))?),
        None => (String::new(), 0),
    };
    Ok((host, num, screen))
}
