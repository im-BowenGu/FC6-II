//! Minimal X11 client: raw wire protocol over a Unix socket, no crates.
//!
//! Frames are rasterised on the CPU and uploaded as ZPixmap PutImage chunks.
//! ConfigureNotify updates the framebuffer when the WM resizes the window.

use std::collections::VecDeque;
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
fn words(body_bytes: usize) -> u16 {
    ((4 + body_bytes) / 4) as u16
}
fn u16at(b: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([b[off], b[off + 1]])
}
fn u32at(b: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([b[off], b[off + 1], b[off + 2], b[off + 3]])
}

fn hex(b: &[u8]) -> String {
    let mut s = String::with_capacity(b.len() * 2);
    for x in b {
        s.push_str(&format!("{x:02x}"));
    }
    s
}

#[derive(Debug)]
#[allow(dead_code)]
pub struct X11 {
    stream: UnixStream,
    pending: Vec<u8>,
    events: Vec<Vec<u8>>,
    replies: VecDeque<Vec<u8>>,
    errors: Vec<Vec<u8>>,
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

#[derive(Default, Debug)]
pub struct Events {
    pub key: bool,
    pub resized: Option<(u16, u16)>,
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
        req[0] = b'l';
        req[2] = 11;
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
                head[0],
                head[2],
                head[3],
                String::from_utf8_lossy(&reason[..reason.len().min(120)])
            ));
        }
        let nwords = u16::from_le_bytes([head[6], head[7]]) as usize;
        let mut body = vec![0u8; nwords * 4];
        stream.read_exact(&mut body).map_err(|e| format!("X setup body: {e}"))?;

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
            events: Vec::new(),
            replies: VecDeque::new(),
            errors: Vec::new(),
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
        if id == 0 { self.base | 1 } else { id }
    }

    fn send(&mut self, b: &[u8]) -> Result<(), String> {
        if env::var_os("VIS_X_DEBUG").is_some() && b.first().copied() != Some(72) {
            let pre = if b.len() >= 4 { hex(&b[..4]) } else { hex(b) };
            eprintln!(
                "XREQ opcode={} bytes={} head={}",
                b.first().copied().unwrap_or(255),
                b.len(),
                pre
            );
        }
        self.stream.write_all(b).map_err(|e| format!("X write: {e}"))
    }

    fn fetch(&mut self, block: bool) -> Result<(), String> {
        let mut buf = [0u8; 8192];
        if block {
            self.stream.set_nonblocking(false).ok();
            match self.stream.read(&mut buf) {
                Ok(0) => return Err("X connection closed".into()),
                Ok(n) => self.pending.extend_from_slice(&buf[..n]),
                Err(e) => return Err(format!("X read: {e}")),
            }
        } else {
            self.stream.set_nonblocking(true).ok();
            loop {
                match self.stream.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => self.pending.extend_from_slice(&buf[..n]),
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                    Err(_) => break,
                }
            }
            self.stream.set_nonblocking(false).ok();
        }
        Ok(())
    }

    fn take_packet(&mut self) -> Option<Vec<u8>> {
        if self.pending.len() < 32 {
            return None;
        }
        let total = if self.pending[0] == 1 {
            if self.pending.len() < 8 {
                return None;
            }
            32 + u32at(&self.pending, 4) as usize * 4
        } else {
            32
        };
        if self.pending.len() < total {
            return None;
        }
        Some(self.pending.drain(..total).collect())
    }

    fn pump(&mut self, block: bool) -> Result<(), String> {
        self.fetch(block)?;
        while let Some(packet) = self.take_packet() {
            match packet[0] {
                0 => self.errors.push(packet),
                1 => self.replies.push_back(packet),
                _ => self.events.push(packet),
            }
        }
        Ok(())
    }

    fn read_reply(&mut self, label: &str) -> Result<Vec<u8>, String> {
        loop {
            if let Some(err) = self.errors.pop() {
                return Ok(err);
            }
            if let Some(reply) = self.replies.pop_front() {
                return Ok(reply);
            }
            self.pump(true).map_err(|e| format!("{label}: {e}"))?;
        }
    }

    fn resource_request(&mut self, opcode: u8, id: u32) -> Result<(), String> {
        let mut b = Vec::with_capacity(8);
        w8(&mut b, opcode);
        w8(&mut b, 0);
        w16(&mut b, words(4));
        w32(&mut b, id);
        self.send(&b)
    }

    pub fn create_window(&mut self, w: u16, h: u16) -> Result<u32, String> {
        let id = self.new_id();
        let mut body = Vec::with_capacity(28);
        w32(&mut body, id);
        w32(&mut body, self.root);
        wi16(&mut body, 60);
        wi16(&mut body, 60);
        w16(&mut body, w);
        w16(&mut body, h);
        w16(&mut body, 0);
        w16(&mut body, 1);
        w32(&mut body, self.visual);
        w32(&mut body, 0);
        let mut b = Vec::with_capacity(32);
        w8(&mut b, 1);
        w8(&mut b, self.depth);
        w16(&mut b, words(body.len()));
        b.extend_from_slice(&body);
        if env::var_os("VIS_X_DEBUG").is_some() {
            eprintln!(
                "CREATE_WINDOW root=0x{:x} visual=0x{:x} depth={} len={} bytes={}",
                self.root,
                self.visual,
                self.depth,
                b.len(),
                hex(&b)
            );
        }
        self.send(&b)?;
        Ok(id)
    }

    pub fn select_events(&mut self, win: u32, event_mask: u32) -> Result<(), String> {
        let mut body = Vec::with_capacity(12);
        w32(&mut body, win);
        w32(&mut body, 1 << 11);
        w32(&mut body, event_mask);
        let mut b = Vec::with_capacity(16);
        w8(&mut b, 2);
        w8(&mut b, 0);
        w16(&mut b, words(body.len()));
        b.extend_from_slice(&body);
        self.send(&b)
    }

    pub fn map_window(&mut self, id: u32) -> Result<(), String> {
        self.resource_request(8, id)
    }

    pub fn create_gc(&mut self, drawable: u32) -> Result<u32, String> {
        let id = self.new_id();
        let mut body = Vec::with_capacity(20);
        w32(&mut body, id);
        w32(&mut body, drawable);
        w32(&mut body, (1 << 2) | (1 << 3));
        w32(&mut body, self.white);
        w32(&mut body, self.black);
        let mut b = Vec::with_capacity(24);
        w8(&mut b, 55);
        w8(&mut b, 0);
        w16(&mut b, words(body.len()));
        b.extend_from_slice(&body);
        self.send(&b)?;
        Ok(id)
    }

    pub fn set_title(&mut self, win: u32, title: &str) -> Result<(), String> {
        let n_units = title.len() as u32;
        let mut data = title.as_bytes().to_vec();
        pad4(&mut data);
        let mut body = Vec::with_capacity(16 + data.len());
        w32(&mut body, win);
        w32(&mut body, 39);
        w32(&mut body, 31);
        w8(&mut body, 8);
        w8(&mut body, 0);
        w16(&mut body, 0);
        w32(&mut body, n_units);
        body.extend_from_slice(&data);
        let mut b = Vec::with_capacity(20 + data.len());
        w8(&mut b, 18);
        w8(&mut b, 0);
        w16(&mut b, words(body.len()));
        b.extend_from_slice(&body);
        self.send(&b)
    }

    pub fn put_image(
        &mut self,
        win: u32,
        gc: u32,
        w: usize,
        h: usize,
        rgba: &[u32],
    ) -> Result<(), String> {
        let bytes_per_row = w * 4;
        let overhead = 24;
        let max_rows = (self.max_request.saturating_sub(overhead) / bytes_per_row).max(1);
        let max_rows = max_rows.min(64);
        let mut y = 0usize;
        while y < h {
            let rows = (h - y).min(max_rows);
            let data_len = rows * bytes_per_row;
            let mut b = Vec::with_capacity(overhead + data_len);
            w8(&mut b, 72);
            w8(&mut b, 2);
            w16(&mut b, words(20 + data_len));
            w32(&mut b, win);
            w32(&mut b, gc);
            w16(&mut b, w as u16);
            w16(&mut b, rows as u16);
            wi16(&mut b, 0);
            wi16(&mut b, y as i16);
            w8(&mut b, 0);
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

    /// Return (width, height, map_state) from GetGeometry/GetWindowAttributes.
    pub fn window_state(&mut self, win: u32) -> Result<(u16, u16, u8), String> {
        self.resource_request(14, win)?;
        let geo = self.read_reply("GetGeometry")?;
        if geo[0] == 0 {
            return Err(format!("GetGeometry error {}", geo[1]));
        }
        let w = u16at(&geo, 16);
        let h = u16at(&geo, 18);
        self.resource_request(3, win)?;
        let attr = self.read_reply("GetWindowAttributes")?;
        if attr[0] == 0 {
            return Err(format!("GetWindowAttributes error {}", attr[1]));
        }
        Ok((w, h, attr[26]))
    }

    pub fn check_errors(&mut self, label: &str) -> Result<(), String> {
        let mut req = [0u8; 4];
        req[0] = 43;
        req[2..4].copy_from_slice(&1u16.to_le_bytes());
        self.send(&req)?;
        let reply = self.read_reply(label)?;
        if reply[0] == 0 {
            return Err(error_text(label, &reply));
        }
        self.pump(false).ok();
        if let Some(err) = self.errors.first() {
            return Err(error_text(label, err));
        }
        self.errors.clear();
        Ok(())
    }

    pub fn poll_events(&mut self, win: u32) -> Events {
        self.pump(false).ok();
        self.errors.clear();
        let mut out = Events::default();
        let queued = std::mem::take(&mut self.events);
        for ev in queued {
            match ev[0] {
                2 | 3 => out.key = true,
                25 if ev.len() >= 24 && u32at(&ev, 8) == win => {
                    let w = u16at(&ev, 20);
                    let h = u16at(&ev, 22);
                    if w > 0 && h > 0 {
                        out.resized = Some((w, h));
                    }
                }
                _ => {}
            }
        }
        out
    }
}

fn error_text(label: &str, rec: &[u8]) -> String {
    format!(
        "{label}: X error code {} seq {} resource 0x{:08x} minor {} major {} raw {}",
        rec[1],
        u16at(rec, 2),
        u32at(rec, 4),
        u16at(rec, 8),
        rec[10],
        hex(rec)
    )
}

fn parse_display(s: &str) -> Result<(String, u32, usize), String> {
    let s = s.trim();
    let (host_num, screen) = match s.rsplit_once('.') {
        Some((a, b)) if b.chars().all(|c| c.is_ascii_digit()) => (a, b.parse().unwrap_or(0)),
        _ => (s, 0),
    };
    let (host, num) = match host_num.rsplit_once(':') {
        Some((h, n)) => (
            h.to_string(),
            n.parse::<u32>()
                .map_err(|_| format!("bad display number in {s}"))?,
        ),
        None => (String::new(), 0),
    };
    Ok((host, num, screen))
}
