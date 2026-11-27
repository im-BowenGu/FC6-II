//! Data readers: binary STL, Taichi velocity slices, HIL GPIO log, CFD lookup.

use std::collections::HashMap;
use std::fs;
use std::fs::File;
use std::io::{BufRead, BufReader, Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

pub struct Stl {
    pub tris: Vec<([f32; 3], [f32; 3], [f32; 3], [f32; 3])>,
}

impl Stl {
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
}

/// One macroscopically averaged D3Q19 slice from the live Taichi solve.
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
        // CSV is written x-major/y-minor: consecutive rows share the same x.
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
        Ok(Self { name: name.to_string(), nx, ny, dx, u })
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

    /// Integrate streamlines through the recorded velocity field (RK2).
    /// Returned points are in physical metres, x increasing left to right.
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

/// Firmware fin-state table mirrored from cfd/fin_states.yaml.
pub const FIN_STATES: [(&str, [f64; 4]); 8] = [
    ("neutral", [0.0, 0.0, 0.0, 0.0]),
    ("pitch_p15", [15.0, 0.0, -15.0, 0.0]),
    ("pitch_n15", [-15.0, 0.0, 15.0, 0.0]),
    ("yaw_p15", [0.0, 15.0, 0.0, -15.0]),
    ("yaw_n15", [0.0, -15.0, 0.0, 15.0]),
    ("roll_p15", [15.0, 15.0, 15.0, 15.0]),
    ("roll_n15", [-15.0, -15.0, -15.0, -15.0]),
    ("xmix_p15", [15.0, -15.0, -15.0, 15.0]),
];

/// Column names must match `hil/logs/bridge.csv` exactly (lower-case).
/// Display code upper-cases them; the tail parser looks them up verbatim.
pub const GPIO_NAMES: [&str; 14] = [
    "pwm_en", "pwm0", "pwm1", "pwm2", "pwm3", "i2c_busy", "spi_busy",
    "sd_cs_n", "usart_tx", "usart_rx", "status_led", "arm_gpio",
    "arm_led", "fault_led",
];

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
    pub gpio: [bool; GPIO_NAMES.len()],
    pub fault_code: f64,
}

pub struct Tail {
    path: PathBuf,
    reader: Option<BufReader<File>>,
    header: Vec<String>,
    idx: HashMap<String, usize>,
    partial: String,
    pub row: TelemetryRow,
    pub updates: u64,
}

impl Tail {
    pub fn new(path: PathBuf, from_start: bool) -> Self {
        Self {
            path,
            reader: None,
            header: Vec::new(),
            idx: HashMap::new(),
            partial: String::new(),
            row: TelemetryRow::default(),
            updates: 0,
        }
        .with_mode(from_start)
    }

    fn with_mode(mut self, from_start: bool) -> Self {
        if let Ok(mut f) = File::open(&self.path) {
            let mut r = BufReader::new(&mut f);
            let mut first = String::new();
            if r.read_line(&mut first).unwrap_or(0) > 0 {
                let fields: Vec<String> = first.trim().split(',').map(|s| s.to_string()).collect();
                self.idx = fields.iter().enumerate().map(|(i, s)| (s.clone(), i)).collect();
                self.header = fields;
            }
            let pos = if from_start { 0 } else { f.seek(SeekFrom::End(0)).unwrap_or(0) };
            let _ = f.seek(SeekFrom::Start(pos));
            self.reader = Some(BufReader::new(f));
        }
        self
    }

    fn get(&self, p: &[&str], name: &str) -> f64 {
        self.idx
            .get(name)
            .and_then(|i| p.get(*i))
            .and_then(|s| s.trim().parse::<f64>().ok())
            .unwrap_or(0.0)
    }

    /// Read newly appended rows. Returns the number of rows consumed.
    pub fn poll(&mut self) -> usize {
        let mut count = 0;
        let mut chunk = String::new();
        if let Some(r) = self.reader.as_mut() {
            let _ = r.read_to_string(&mut chunk);
        } else {
            *self = Self::new(self.path.clone(), true);
            return 0;
        }
        self.partial.push_str(&chunk);
        loop {
            let Some(pos) = self.partial.find('\n') else { break };
            let line: String = self.partial.drain(..=pos).collect();
            let line = line.trim();
            if line.is_empty() || line.starts_with("t_ms") {
                continue;
            }
            let p: Vec<&str> = line.split(',').collect();
            if p.len() < self.header.len().saturating_sub(2) {
                continue;
            }
            let mut row = TelemetryRow {
                t_ms: self.get(&p, "t_ms"),
                state: self.get(&p, "state"),
                alt_m: self.get(&p, "alt_m"),
                pitch_deg: self.get(&p, "pitch_deg"),
                roll_deg: self.get(&p, "roll_deg"),
                pred_apo_m: self.get(&p, "pred_apo_m"),
                vv_ms: self.get(&p, "vv_ms"),
                fault_code: self.get(&p, "fault_code"),
                ..Default::default()
            };
            for i in 0..4 {
                row.fins[i] = self.get(&p, &format!("fin{i}"));
                row.pwm_us[i] = self.get(&p, &format!("pwm{i}_us"));
            }
            for (i, name) in GPIO_NAMES.iter().enumerate() {
                row.gpio[i] = self.get(&p, name) > 0.5;
            }
            self.row = row;
            count += 1;
        }
        self.updates += count as u64;
        count
    }
}

/// Read the calibrated neutral drag coefficient from the recorded lookup.
pub fn neutral_cd(path: &Path) -> f64 {
    let Ok(text) = fs::read_to_string(path) else { return 1.01068 };
    let Some(pos) = text.find("\"name\": \"neutral\"") else { return 1.01068 };
    let rest = &text[pos..];
    let Some(c) = rest.find("\"Cd\":") else { return 1.01068 };
    rest[c + 5..]
        .trim_start()
        .split([',', '}'])
        .next()
        .and_then(|s| s.trim().parse::<f64>().ok())
        .unwrap_or(1.01068)
}
