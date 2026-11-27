//! Minimal JSON reader and per-rocket visualiser configuration.

use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

#[allow(dead_code)]
#[derive(Clone, Debug)]
pub enum Json {
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<Json>),
    Obj(Vec<(String, Json)>),
}

impl Json {
    pub fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(fields) => fields.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }

    pub fn text(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s),
            _ => None,
        }
    }

    pub fn num(&self) -> Option<f64> {
        match self {
            Json::Num(v) => Some(*v),
            _ => None,
        }
    }

    pub fn usize(&self) -> Option<usize> {
        self.num().and_then(|v| {
            if v >= 0.0 && v.fract() == 0.0 && v <= usize::MAX as f64 {
                Some(v as usize)
            } else {
                None
            }
        })
    }

    pub fn items(&self) -> Option<&[Json]> {
        match self {
            Json::Arr(items) => Some(items),
            _ => None,
        }
    }

    pub fn fields(&self) -> Option<&[(String, Json)]> {
        match self {
            Json::Obj(fields) => Some(fields),
            _ => None,
        }
    }
}

struct Reader<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn ws(&mut self) {
        while self.pos < self.bytes.len() && self.bytes[self.pos].is_ascii_whitespace() {
            self.pos += 1;
        }
    }

    fn value(&mut self) -> Result<Json, String> {
        self.ws();
        let c = *self.bytes.get(self.pos).ok_or("unexpected end of JSON")?;
        match c {
            b'{' => self.object(),
            b'[' => self.array(),
            b'"' => Ok(Json::Str(self.string()?)),
            b't' => {
                self.literal(b"true")?;
                Ok(Json::Bool(true))
            }
            b'f' => {
                self.literal(b"false")?;
                Ok(Json::Bool(false))
            }
            b'n' => {
                self.literal(b"null")?;
                Ok(Json::Null)
            }
            b'-' | b'0'..=b'9' => self.number(),
            _ => Err(format!("unexpected byte {:?} at {}", c as char, self.pos)),
        }
    }

    fn literal(&mut self, word: &[u8]) -> Result<(), String> {
        if self.bytes.get(self.pos..self.pos + word.len()) == Some(word) {
            self.pos += word.len();
            Ok(())
        } else {
            Err(format!("bad literal at {}", self.pos))
        }
    }

    fn number(&mut self) -> Result<Json, String> {
        let start = self.pos;
        while self.pos < self.bytes.len()
            && matches!(self.bytes[self.pos], b'-' | b'+' | b'.' | b'e' | b'E' | b'0'..=b'9')
        {
            self.pos += 1;
        }
        let text = std::str::from_utf8(&self.bytes[start..self.pos]).map_err(|e| e.to_string())?;
        text.parse::<f64>()
            .map(Json::Num)
            .map_err(|e| format!("bad number at {start}: {e}"))
    }

    fn string(&mut self) -> Result<String, String> {
        if self.bytes.get(self.pos) != Some(&b'"') {
            return Err(format!("expected string at {}", self.pos));
        }
        self.pos += 1;
        let mut out = String::new();
        loop {
            let c = *self.bytes.get(self.pos).ok_or("unterminated string")?;
            self.pos += 1;
            match c {
                b'"' => return Ok(out),
                b'\\' => {
                    let esc = *self.bytes.get(self.pos).ok_or("bad escape")?;
                    self.pos += 1;
                    match esc {
                        b'"' => out.push('"'),
                        b'\\' => out.push('\\'),
                        b'/' => out.push('/'),
                        b'b' => out.push('\u{0008}'),
                        b'f' => out.push('\u{000c}'),
                        b'n' => out.push('\n'),
                        b'r' => out.push('\r'),
                        b't' => out.push('\t'),
                        b'u' => {
                            let mut cp = self.hex4()? as u32;
                            if (0xd800..=0xdbff).contains(&cp)
                                && self.bytes.get(self.pos..self.pos + 2) == Some(b"\\u")
                            {
                                self.pos += 2;
                                let lo = self.hex4()? as u32;
                                if (0xdc00..=0xdfff).contains(&lo) {
                                    cp = 0x10000 + ((cp - 0xd800) << 10) + (lo - 0xdc00);
                                }
                            }
                            out.push(char::from_u32(cp).unwrap_or('\u{fffd}'));
                        }
                        _ => return Err(format!("bad escape at {}", self.pos - 1)),
                    }
                }
                _ => out.push(c as char),
            }
        }
    }

    fn hex4(&mut self) -> Result<u16, String> {
        let end = self.pos + 4;
        let text = self
            .bytes
            .get(self.pos..end)
            .ok_or("truncated unicode escape")?;
        let text = std::str::from_utf8(text).map_err(|e| e.to_string())?;
        self.pos = end;
        u16::from_str_radix(text, 16).map_err(|e| format!("bad unicode escape: {e}"))
    }

    fn array(&mut self) -> Result<Json, String> {
        self.pos += 1;
        let mut items = Vec::new();
        loop {
            self.ws();
            if self.bytes.get(self.pos) == Some(&b']') {
                self.pos += 1;
                return Ok(Json::Arr(items));
            }
            items.push(self.value()?);
            self.ws();
            match self.bytes.get(self.pos) {
                Some(b',') => self.pos += 1,
                Some(b']') => {
                    self.pos += 1;
                    return Ok(Json::Arr(items));
                }
                _ => return Err(format!("expected ',' or ']' at {}", self.pos)),
            }
        }
    }

    fn object(&mut self) -> Result<Json, String> {
        self.pos += 1;
        let mut fields = Vec::new();
        loop {
            self.ws();
            if self.bytes.get(self.pos) == Some(&b'}') {
                self.pos += 1;
                return Ok(Json::Obj(fields));
            }
            let key = self.string()?;
            self.ws();
            if self.bytes.get(self.pos) != Some(&b':') {
                return Err(format!("expected ':' at {}", self.pos));
            }
            self.pos += 1;
            let value = self.value()?;
            fields.push((key, value));
            self.ws();
            match self.bytes.get(self.pos) {
                Some(b',') => self.pos += 1,
                Some(b'}') => {
                    self.pos += 1;
                    return Ok(Json::Obj(fields));
                }
                _ => return Err(format!("expected ',' or '}}' at {}", self.pos)),
            }
        }
    }
}

pub fn parse(text: &str) -> Result<Json, String> {
    let mut reader = Reader {
        bytes: text.as_bytes(),
        pos: 0,
    };
    let value = reader.value()?;
    reader.ws();
    if reader.pos != reader.bytes.len() {
        return Err(format!("trailing JSON data at {}", reader.pos));
    }
    Ok(value)
}

#[derive(Clone, Debug)]
pub struct Paths {
    pub log: PathBuf,
    pub stl: PathBuf,
    pub fields: PathBuf,
    pub motion: PathBuf,
    pub socket: PathBuf,
}

#[derive(Clone, Debug)]
pub struct FinState {
    pub name: String,
    pub angles: [f64; 4],
}

#[derive(Clone, Debug)]
pub struct Geometry {
    pub cg_x: f64,
    pub hinge_x: f64,
    pub hinge_r: f64,
    pub x_min: f64,
    pub x_max: f64,
    pub y_min: f64,
    pub y_max: f64,
    pub grid_m: f64,
}

impl Geometry {
    pub fn from_bounds(min: [f64; 3], max: [f64; 3]) -> Self {
        let len = (max[0] - min[0]).max(1e-3);
        let radius = (max[1] - min[1]).max(max[2] - min[2]).max(1e-3);
        Self {
            cg_x: min[0] + len * 0.55,
            hinge_x: max[0] - len * 0.18,
            hinge_r: radius * 0.18,
            x_min: min[0] - len * 0.05,
            x_max: max[0] + len * 0.05,
            y_min: -radius * 0.6,
            y_max: radius * 0.6,
            grid_m: len / 10.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Config {
    pub rocket: String,
    pub title: String,
    pub w: usize,
    pub h: usize,
    pub paths: Paths,
    pub gpio_names: Vec<String>,
    pub fin_states: Vec<FinState>,
    pub labels: HashMap<String, String>,
    pub geometry: Option<Geometry>,
    pub dir: PathBuf,
}

impl Config {
    pub fn label(&self, key: &str) -> &str {
        self.labels
            .get(key)
            .map(String::as_str)
            .unwrap_or_else(|| fallback_label(key))
    }
}

fn fallback_label(key: &str) -> &'static str {
    match key {
        "source" => "SOURCE",
        "live" => "LIVE",
        "replay" => "REPLAY",
        "stale" => "STALE",
        "cfd" => "CFD",
        "telemetry" => "TELEMETRY",
        "feed" => "FEED",
        "time" => "TIME",
        "alt" => "ALT",
        "vv" => "VV",
        "pred_apo" => "PRED APO",
        "pitch" => "PITCH",
        "roll" => "ROLL",
        "fault" => "FAULT",
        "fin_act" => "FIN ACT",
        "pwm" => "PWM",
        "gpio" => "GPIO",
        "seq" => "SEQ",
        "hz" => "HZ",
        "field" => "FIELD",
        "forces" => "FORCES",
        "state" => "STATE",
        "empty_cfd" => "NO FIELD",
        "quit" => "KEY=QUIT",
        _ => "",
    }
}

fn need_str(value: Option<&Json>, key: &str) -> Result<String, String> {
    value
        .and_then(Json::text)
        .map(str::to_string)
        .ok_or_else(|| format!("missing string '{key}'"))
}

fn need_num(value: Option<&Json>, key: &str) -> Result<f64, String> {
    value
        .and_then(Json::num)
        .ok_or_else(|| format!("missing number '{key}'"))
}

fn need_usize(value: Option<&Json>, key: &str) -> Result<usize, String> {
    value
        .and_then(Json::usize)
        .ok_or_else(|| format!("missing positive integer '{key}'"))
}

pub fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../..")
        .canonicalize()
        .unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../.."))
}

pub fn resolve(raw: &Path, base: &Path, repo: &Path) -> PathBuf {
    if raw.is_absolute() {
        return raw.to_path_buf();
    }
    let in_rocket = base.join(raw);
    if in_rocket.exists() {
        return in_rocket;
    }
    let in_repo = repo.join(raw);
    if in_repo.exists() {
        return in_repo;
    }
    in_rocket
}

pub fn load(path: &Path) -> Result<Config, String> {
    let text = fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let root = parse(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    let repo = repo_root();
    let dir = path
        .canonicalize()
        .unwrap_or_else(|_| path.to_path_buf())
        .parent()
        .unwrap_or(Path::new("."))
        .to_path_buf();
    let base = dir.parent().unwrap_or(&dir).to_path_buf();

    let paths_obj = root.get("paths").ok_or("missing 'paths' object")?;
    let path_str = |key: &str| need_str(paths_obj.get(key), &format!("paths.{key}"));
    let paths = Paths {
        log: resolve(Path::new(&path_str("log")?), &base, &repo),
        stl: resolve(Path::new(&path_str("stl")?), &base, &repo),
        fields: resolve(Path::new(&path_str("fields")?), &base, &repo),
        motion: resolve(Path::new(&path_str("motion")?), &base, &repo),
        socket: resolve(Path::new(&path_str("socket")?), &base, &repo),
    };

    let gpio_names = root
        .get("gpio_names")
        .and_then(Json::items)
        .ok_or("missing 'gpio_names' array")?
        .iter()
        .enumerate()
        .map(|(i, v)| {
            need_str(Some(v), &format!("gpio_names[{i}]"))
        })
        .collect::<Result<Vec<_>, _>>()?;

    let fin_states = root
        .get("fin_states")
        .and_then(Json::items)
        .ok_or("missing 'fin_states' array")?
        .iter()
        .enumerate()
        .map(|(i, item)| {
            let name = need_str(item.get("name"), &format!("fin_states[{i}].name"))?;
            let angles = if let Some(values) = item.get("angles").and_then(Json::items) {
                if values.len() != 4 {
                    return Err(format!("fin_states[{i}].angles must have 4 values"));
                }
                let mut out = [0.0; 4];
                for (j, value) in values.iter().enumerate() {
                    out[j] = need_num(Some(value), &format!("fin_states[{i}].angles[{j}]"))?;
                }
                out
            } else {
                let mut out = [0.0; 4];
                for j in 0..4 {
                    out[j] = need_num(item.get(&format!("fin{j}")), &format!("fin_states[{i}].fin{j}"))?;
                }
                out
            };
            Ok(FinState { name, angles })
        })
        .collect::<Result<Vec<_>, _>>()?;

    let mut labels = HashMap::new();
    if let Some(fields) = root.get("labels").and_then(Json::fields) {
        for (key, value) in fields {
            if let Some(text) = value.text() {
                labels.insert(key.clone(), text.to_string());
            }
        }
    }

    let geometry = root.get("geometry").map(|g| Geometry {
        cg_x: g.get("cg_x").and_then(Json::num).unwrap_or(0.5),
        hinge_x: g.get("hinge_x").and_then(Json::num).unwrap_or(1.0),
        hinge_r: g.get("hinge_r").and_then(Json::num).unwrap_or(0.01),
        x_min: g.get("x_min").and_then(Json::num).unwrap_or(-0.1),
        x_max: g.get("x_max").and_then(Json::num).unwrap_or(1.1),
        y_min: g.get("y_min").and_then(Json::num).unwrap_or(-0.5),
        y_max: g.get("y_max").and_then(Json::num).unwrap_or(0.5),
        grid_m: g.get("grid_m").and_then(Json::num).unwrap_or(0.1).max(1e-6),
    });

    Ok(Config {
        rocket: need_str(root.get("rocket_name"), "rocket_name")?,
        title: need_str(root.get("title"), "title")?,
        w: need_usize(root.get("width"), "width")?.max(320),
        h: need_usize(root.get("height"), "height")?.max(240),
        paths,
        gpio_names,
        fin_states,
        labels,
        geometry,
        dir,
    })
}
