//! Data readers and a bounded, reconnecting CFD socket client.

use crate::config::{self, Geometry, Json};
use std::collections::{BTreeMap, HashMap, VecDeque};
use std::fs::{self, File};
use std::io::{BufRead, BufReader, Read, Seek, SeekFrom};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const MAGIC: &[u8; 4] = b"CFD1";
const MAX_JSON: usize = 1 << 20;
const MAX_CELLS: usize = 4_000_000;
const MAX_BUFFER: usize = 32 << 20;
const READS_PER_POLL: usize = 16;
const BYTES_PER_POLL: usize = 2 << 20;
const STALE_AFTER: Duration = Duration::from_millis(1500);
const RETRY_DELAY: Duration = Duration::from_millis(350);
const HISTORY: usize = 16;

pub struct Stl {
    pub tris: Vec<([f32; 3], [f32; 3], [f32; 3], [f32; 3])>,
}

impl Stl {
    pub fn empty() -> Self {
        Self { tris: Vec::new() }
    }

    pub fn load(path: &Path) -> Result<Self, String> {
        let d = fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
        if d.len() < 84 {
            return Err(format!("{}: too short for binary STL", path.display()));
        }
        let n = u32::from_le_bytes([d[80], d[81], d[82], d[83]]) as usize;
        if 84 + 50 * n > d.len() {
            return Err(format!("{}: STL count {n} exceeds file size", path.display()));
        }
        let mut tris = Vec::with_capacity(n);
        for i in 0..n {
            let o = 84 + 50 * i;
            let mut f = [0f32; 12];
            for k in 0..12 {
                let p = o + k * 4;
                f[k] = f32::from_le_bytes([d[p], d[p + 1], d[p + 2], d[p + 3]]);
            }
            tris.push((
                [f[0], f[1], f[2]],
                [f[3], f[4], f[5]],
                [f[6], f[7], f[8]],
                [f[9], f[10], f[11]],
            ));
        }
        Ok(Self { tris })
    }

    pub fn bounds(&self) -> ([f64; 3], [f64; 3]) {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for (_, a, b, c) in &self.tris {
            for p in [a, b, c] {
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k] as f64);
                    hi[k] = hi[k].max(p[k] as f64);
                }
            }
        }
        if !lo[0].is_finite() {
            return ([0.0, -0.5, -0.5], [1.0, 0.5, 0.5]);
        }
        (lo, hi)
    }

    pub fn geometry(&self) -> Geometry {
        let (lo, hi) = self.bounds();
        Geometry::from_bounds(lo, hi)
    }
}

#[derive(Clone)]
pub struct Field {
    pub name: String,
    pub nx: usize,
    pub ny: usize,
    pub dx: f32,
    pub u: Vec<[f32; 3]>,
}

impl Field {
    pub fn load(path: &Path, name: &str) -> Result<Self, String> {
        let fh = File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let mut rdr = BufReader::new(fh);
        let mut line = String::new();
        rdr.read_line(&mut line).map_err(|e| e.to_string())?;
        let mut rows: Vec<(f32, f32, [f32; 3])> = Vec::new();
        loop {
            line.clear();
            if rdr.read_line(&mut line).map_err(|e| e.to_string())? == 0 {
                break;
            }
            let p: Vec<&str> = line.trim().split(',').collect();
            if p.len() < 5 {
                continue;
            }
            let v = |s: &str| s.parse::<f32>().unwrap_or(0.0);
            rows.push((v(p[0]), v(p[1]), [v(p[2]), v(p[3]), v(p[4])]));
        }
        if rows.len() < 64 {
            return Err(format!("{}: too few field rows", path.display()));
        }
        let first_x = rows[0].0;
        let mut ny = 0usize;
        while ny < rows.len() && rows[ny].0 == first_x {
            ny += 1;
        }
        if ny == 0 || rows.len() % ny != 0 || ny < 8 {
            return Err(format!("{}: irregular field grid", path.display()));
        }
        let nx = rows.len() / ny;
        if nx < 16 {
            return Err(format!("{}: incomplete field grid", path.display()));
        }
        let dx = rows[ny].0 - rows[0].0;
        if dx <= 0.0 || !dx.is_finite() {
            return Err(format!("{}: bad grid spacing", path.display()));
        }
        let u = rows.into_iter().map(|(_, _, v)| v).collect();
        Ok(Self {
            name: name.to_string(),
            nx,
            ny,
            dx,
            u,
        })
    }

    #[inline]
    fn at(&self, x: usize, y: usize) -> [f32; 3] {
        self.u[(x.min(self.nx - 1)) * self.ny + y.min(self.ny - 1)]
    }

    fn sample(&self, fx: f32, fy: f32) -> [f32; 3] {
        let x0 = fx.floor() as usize;
        let y0 = fy.floor() as usize;
        let tx = fx - x0 as f32;
        let ty = fy - y0 as f32;
        let mut out = [0f32; 3];
        for k in 0..3 {
            let a = self.at(x0, y0)[k] * (1.0 - tx) + self.at(x0 + 1, y0)[k] * tx;
            let b = self.at(x0, y0 + 1)[k] * (1.0 - tx) + self.at(x0 + 1, y0 + 1)[k] * tx;
            out[k] = a * (1.0 - ty) + b * ty;
        }
        out
    }

    /// Integrate streamlines through the velocity field (RK2), in metres.
    pub fn streamlines(&self) -> Vec<Vec<(f32, f32, f32)>> {
        let mut out = Vec::new();
        let step = 0.8f32;
        for sy in (4..self.ny.saturating_sub(4)).step_by(5) {
            let mut x = 2.0f32;
            let mut y = sy as f32;
            let mut line = Vec::new();
            for _ in 0..900 {
                if x >= self.nx as f32 - 2.0 || y < 1.0 || y > self.ny as f32 - 2.0 {
                    break;
                }
                let v = self.sample(x, y);
                let mag = (v[0] * v[0] + v[1] * v[1]).sqrt();
                if mag < 1e-5 {
                    break;
                }
                let ux = v[0] / mag;
                let uy = v[1] / mag;
                let mx = self.sample(x + ux * step * 0.5, y + uy * step * 0.5);
                let mm = (mx[0] * mx[0] + mx[1] * mx[1]).sqrt().max(1e-6);
                x += mx[0] / mm * step;
                y += mx[1] / mm * step;
                line.push((
                    (x + 0.5) * self.dx,
                    (y - self.ny as f32 * 0.5) * self.dx,
                    mag,
                ));
            }
            if line.len() > 8 {
                out.push(line);
            }
        }
        out
    }
}

#[derive(Clone)]
pub struct LiveFrame {
    pub seq: u64,
    pub t_ms: f64,
    pub nx: usize,
    pub ny: usize,
    pub dx_m: f64,
    pub angles: [f64; 4],
    pub forces: BTreeMap<String, f64>,
    pub state: String,
    pub u: Vec<[f32; 3]>,
}

pub struct LiveField {
    path: PathBuf,
    name: String,
    stream: Option<UnixStream>,
    buf: Vec<u8>,
    next_connect: Instant,
    latest: Option<LiveFrame>,
    latest_field: Option<Field>,
    last_rx: Instant,
    pub frames: u64,
    pub hz: f64,
}

impl LiveField {
    pub fn new(path: PathBuf, name: String) -> Self {
        Self {
            path,
            name,
            stream: None,
            buf: Vec::new(),
            next_connect: Instant::now(),
            latest: None,
            latest_field: None,
            last_rx: Instant::now(),
            frames: 0,
            hz: 0.0,
        }
    }

    pub fn frame(&self) -> Option<&LiveFrame> {
        self.latest.as_ref()
    }

    pub fn field(&self) -> Option<&Field> {
        self.latest_field.as_ref()
    }

    pub fn fresh(&self, now: Instant) -> bool {
        self.latest.is_some() && now.duration_since(self.last_rx) < STALE_AFTER
    }

    /// Non-blocking frame read. Short reads are buffered; failures reconnect.
    pub fn recv(&mut self) -> Option<&LiveFrame> {
        let now = Instant::now();
        if self.stream.is_none() && now >= self.next_connect {
            match UnixStream::connect(&self.path) {
                Ok(stream) => {
                    stream.set_nonblocking(true).ok();
                    self.stream = Some(stream);
                    self.buf.clear();
                }
                Err(_) => self.next_connect = now + RETRY_DELAY,
            }
        }

        let mut dropped = false;
        if let Some(stream) = self.stream.as_mut() {
            let mut chunk = [0u8; 16384];
            let mut reads = 0;
            let mut got = 0;
            while reads < READS_PER_POLL && got < BYTES_PER_POLL {
                match stream.read(&mut chunk) {
                    Ok(0) => {
                        dropped = true;
                        break;
                    }
                    Ok(n) => {
                        self.buf.extend_from_slice(&chunk[..n]);
                        got += n;
                        reads += 1;
                        if self.buf.len() > MAX_BUFFER {
                            dropped = true;
                            break;
                        }
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                    Err(_) => {
                        dropped = true;
                        break;
                    }
                }
            }
        }
        if dropped {
            self.disconnect(now);
        }

        loop {
            match self.take_frame() {
                Ok(Some(frame)) => self.accept(frame, now),
                Ok(None) => break,
                Err(_) => {
                    self.disconnect(now);
                    break;
                }
            }
        }
        self.latest.as_ref()
    }

    fn disconnect(&mut self, now: Instant) {
        self.stream.take();
        self.buf.clear();
        self.next_connect = now + RETRY_DELAY;
    }

    fn take_frame(&mut self) -> Result<Option<LiveFrame>, String> {
        if self.buf.len() < 4 {
            return Ok(None);
        }
        if &self.buf[..4] != MAGIC {
            return Err("bad CFD frame magic".into());
        }
        if self.buf.len() < 8 {
            return Ok(None);
        }
        let json_len = u32::from_le_bytes(self.buf[4..8].try_into().unwrap()) as usize;
        if json_len == 0 || json_len > MAX_JSON {
            return Err("implausible CFD metadata length".into());
        }
        let meta_end = 8 + json_len;
        if self.buf.len() < meta_end {
            return Ok(None);
        }
        let text = std::str::from_utf8(&self.buf[8..meta_end]).map_err(|e| e.to_string())?;
        let meta = config::parse(text)?;
        let number = |key: &str| -> Result<f64, String> {
            meta.get(key)
                .and_then(Json::num)
                .ok_or_else(|| format!("missing CFD field '{key}'"))
        };
        let count = |key: &str| -> Result<usize, String> {
            let v = number(key)?;
            if v < 1.0 || v.fract() != 0.0 || v > MAX_CELLS as f64 {
                return Err(format!("bad CFD grid dimension '{key}'"));
            }
            Ok(v as usize)
        };
        let nx = count("nx")?;
        let ny = count("ny")?;
        let cells = nx.checked_mul(ny).ok_or("CFD grid overflow")?;
        if cells > MAX_CELLS {
            return Err("CFD grid too large".into());
        }
        let dx_m = number("dx_m")?;
        if dx_m <= 0.0 || !dx_m.is_finite() {
            return Err("bad CFD grid spacing".into());
        }
        let frame_end = meta_end + cells * 12;
        if self.buf.len() < frame_end {
            return Ok(None);
        }

        let mut angles = [0f64; 4];
        if let Some(items) = meta.get("angles").and_then(Json::items) {
            for (i, item) in items.iter().take(4).enumerate() {
                angles[i] = item.num().unwrap_or(0.0);
            }
        }
        let mut forces = BTreeMap::new();
        if let Some(Json::Obj(fields)) = meta.get("forces") {
            for (key, value) in fields {
                if let Some(v) = value.num() {
                    forces.insert(key.clone(), v);
                }
            }
        }
        let state = meta
            .get("state")
            .and_then(Json::text)
            .unwrap_or("")
            .to_string();

        let mut u = Vec::with_capacity(cells);
        for i in 0..cells {
            let o = meta_end + i * 12;
            u.push([
                f32::from_le_bytes(self.buf[o..o + 4].try_into().unwrap()),
                f32::from_le_bytes(self.buf[o + 4..o + 8].try_into().unwrap()),
                f32::from_le_bytes(self.buf[o + 8..o + 12].try_into().unwrap()),
            ]);
        }
        self.buf.drain(..frame_end);

        Ok(Some(LiveFrame {
            seq: number("seq")? as u64,
            t_ms: number("t_ms")?,
            nx,
            ny,
            dx_m,
            angles,
            forces,
            state,
            u,
        }))
    }

    fn accept(&mut self, frame: LiveFrame, now: Instant) {
        self.frames += 1;
        if let Some(prev) = self.latest.as_ref() {
            let dt = now.duration_since(self.last_rx).as_secs_f64();
            if dt > 0.0 && frame.seq > prev.seq {
                let instant = (frame.seq - prev.seq) as f64 / dt;
                self.hz = if self.hz == 0.0 {
                    instant
                } else {
                    self.hz * 0.7 + instant * 0.3
                };
            }
        }
        let field = Field {
            name: self.name.clone(),
            nx: frame.nx,
            ny: frame.ny,
            dx: frame.dx_m as f32,
            u: frame.u.clone(),
        };
        self.latest = Some(frame);
        self.latest_field = Some(field);
        self.last_rx = now;
    }
}

#[derive(Clone, Default)]
pub struct TelemetryRow {
    pub t_ms: f64,
    pub state: f64,
    pub alt_m: f64,
    pub pitch_deg: f64,
    pub roll_deg: f64,
    pub pred_apo_m: f64,
    pub vv_ms: f64,
    pub fins: [f64; 4],
    pub pwm_us: [f64; 4],
    pub gpio: Vec<bool>,
    pub fault_code: f64,
}

impl TelemetryRow {
    fn with_gpio(count: usize) -> Self {
        let mut row = Self::default();
        row.gpio = vec![false; count];
        row
    }
}

pub struct Tail {
    path: PathBuf,
    reader: Option<BufReader<File>>,
    idx: HashMap<String, usize>,
    partial: String,
    gpio_names: Vec<String>,
    pub row: TelemetryRow,
    pub history: VecDeque<TelemetryRow>,
    pub updates: u64,
}

impl Tail {
    pub fn new(path: PathBuf, gpio_names: &[String], from_start: bool) -> Self {
        let mut tail = Self {
            path,
            reader: None,
            idx: HashMap::new(),
            partial: String::new(),
            gpio_names: gpio_names.to_vec(),
            row: TelemetryRow::with_gpio(gpio_names.len()),
            history: VecDeque::with_capacity(HISTORY),
            updates: 0,
        };
        tail.open(from_start);
        tail
    }

    fn open(&mut self, from_start: bool) {
        let Ok(mut file) = File::open(&self.path) else { return };
        {
            let mut probe = BufReader::new(&mut file);
            let mut first = String::new();
            if probe.read_line(&mut first).unwrap_or(0) > 0 {
                self.idx = first
                    .trim()
                    .split(',')
                    .enumerate()
                    .map(|(i, name)| (name.trim().to_ascii_lowercase(), i))
                    .collect();
            }
        }
        let pos = if from_start {
            0
        } else {
            file.seek(SeekFrom::End(0)).unwrap_or(0)
        };
        let _ = file.seek(SeekFrom::Start(pos));
        self.reader = Some(BufReader::new(file));
    }

    fn get(&self, parts: &[&str], name: &str) -> f64 {
        let key = name.to_ascii_lowercase();
        self.idx
            .get(&key)
            .and_then(|i| parts.get(*i))
            .and_then(|s| s.trim().parse::<f64>().ok())
            .unwrap_or(0.0)
    }

    pub fn poll(&mut self) -> Vec<TelemetryRow> {
        let Some(reader) = self.reader.as_mut() else {
            self.open(true);
            return Vec::new();
        };
        let mut chunk = String::new();
        let _ = reader.read_to_string(&mut chunk);
        self.partial.push_str(&chunk);

        let mut fresh = Vec::new();
        while let Some(pos) = self.partial.find('\n') {
            let line: String = self.partial.drain(..=pos).collect();
            let line = line.trim();
            if line.is_empty() || line.to_ascii_lowercase().starts_with("t_ms") {
                continue;
            }
            let parts: Vec<&str> = line.split(',').collect();
            if parts.len() < self.idx.len().saturating_sub(2) {
                continue;
            }
            let mut row = TelemetryRow {
                t_ms: self.get(&parts, "t_ms"),
                state: self.get(&parts, "state"),
                alt_m: self.get(&parts, "alt_m"),
                pitch_deg: self.get(&parts, "pitch_deg"),
                roll_deg: self.get(&parts, "roll_deg"),
                pred_apo_m: self.get(&parts, "pred_apo_m"),
                vv_ms: self.get(&parts, "vv_ms"),
                fault_code: self.get(&parts, "fault_code"),
                ..TelemetryRow::with_gpio(self.gpio_names.len())
            };
            for i in 0..4 {
                row.fins[i] = self.get(&parts, &format!("fin{i}"));
                row.pwm_us[i] = self.get(&parts, &format!("pwm{i}_us"));
            }
            for i in 0..self.gpio_names.len() {
                row.gpio[i] = self.get(&parts, &self.gpio_names[i]) > 0.5;
            }
            self.row = row.clone();
            self.history.push_back(row.clone());
            while self.history.len() > HISTORY {
                self.history.pop_front();
            }
            fresh.push(row);
        }
        self.updates += fresh.len() as u64;
        fresh
    }
}

pub fn neutral_cd(path: &Path) -> Option<f64> {
    let text = fs::read_to_string(path).ok()?;
    let root = config::parse(&text).ok()?;
    if let Some(samples) = root.get("samples").and_then(Json::items) {
        for sample in samples {
            if let Some(cd) = sample.get("Cd").and_then(Json::num) {
                return Some(cd);
            }
        }
    }
    root.get("Cd").and_then(Json::num)
}
