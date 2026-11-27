//! Cross-rocket monochrome X11 visualiser for CFD fields and telemetry.

mod config;
mod draw;
mod model;
mod x11;

use config::{Config, Geometry};
use draw::{rgb, shade, Frame};
use model::{Field, LiveField, LiveFrame, Stl, Tail, TelemetryRow};
use std::collections::VecDeque;
use std::env;
use std::fs::{self, File};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const DEFAULT_CONFIG: &str = "config/vis.json";

const BG: u32 = rgb(9, 9, 9);
const PANEL: u32 = rgb(16, 16, 16);
const GRID: u32 = rgb(50, 50, 50);
const TEXT: u32 = rgb(224, 224, 224);
const DIM: u32 = rgb(132, 133, 134);
const WHITE: u32 = rgb(248, 248, 248);
const AMBER: u32 = rgb(255, 194, 112);

#[derive(Default)]
struct Overrides {
    log: Option<PathBuf>,
    stl: Option<PathBuf>,
    fields: Option<PathBuf>,
    motion: Option<PathBuf>,
    socket: Option<PathBuf>,
}

struct Args {
    config: PathBuf,
    replay: bool,
    fps: u32,
    snapshot: Option<PathBuf>,
    snapshot_after: f64,
    print_rows: bool,
    hold: bool,
    overrides: Overrides,
}

fn default_args(_repo: &Path) -> Args {
    Args {
        config: PathBuf::from(DEFAULT_CONFIG),
        replay: false,
        fps: 30,
        snapshot: None,
        snapshot_after: 1.0,
        print_rows: false,
        hold: false,
        overrides: Overrides::default(),
    }
}

fn resolve_config(raw: PathBuf, repo: &Path) -> PathBuf {
    if raw.is_absolute() || raw.exists() {
        return raw;
    }
    let mut trimmed = raw.to_string_lossy().to_string();
    loop {
        if let Some(rest) = trimmed.strip_prefix("../") {
            trimmed = rest.to_string();
            let candidate = repo.join(&trimmed);
            if candidate.exists() {
                return candidate;
            }
        } else {
            break;
        }
    }
    let candidate = repo.join(&raw);
    if candidate.exists() {
        candidate
    } else {
        raw
    }
}

fn parse_args() -> Args {
    let repo = config::repo_root();
    let mut args = default_args(&repo);
    let mut it = env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--config" => args.config = PathBuf::from(it.next().unwrap_or_default()),
            "--replay" => args.replay = true,
            "--fps" => {
                args.fps = it
                    .next()
                    .and_then(|v| v.parse().ok())
                    .unwrap_or(30)
                    .clamp(1, 120);
            }
            "--log" => args.overrides.log = Some(PathBuf::from(it.next().unwrap_or_default())),
            "--stl" => args.overrides.stl = Some(PathBuf::from(it.next().unwrap_or_default())),
            "--fields" => {
                args.overrides.fields = Some(PathBuf::from(it.next().unwrap_or_default()))
            }
            "--motion" => {
                args.overrides.motion = Some(PathBuf::from(it.next().unwrap_or_default()))
            }
            "--socket" => {
                args.overrides.socket = Some(PathBuf::from(it.next().unwrap_or_default()))
            }
            "--snapshot" => {
                args.snapshot = Some(PathBuf::from(it.next().unwrap_or_default()));
            }
            "--snapshot-after" => {
                let secs: f64 = it.next().and_then(|v| v.parse().ok()).unwrap_or(1.0);
                args.snapshot_after = secs.max(0.0);
            }
            "--print-rows" => args.print_rows = true,
            "--hold" => args.hold = true,
            "-h" | "--help" => {
                println!(
                    "rocket-vis --config PATH [--replay] [--fps N] \
                     [--log PATH] [--stl PATH] [--fields DIR] [--motion PATH] [--socket PATH]\n\
                     [ --snapshot PPM [--snapshot-after SECS] ] [--print-rows] [--hold]"
                );
                std::process::exit(0);
            }
            _ => eprintln!("unknown argument: {arg}"),
        }
    }
    args
}

fn main() {
    let args = parse_args();
    let repo = config::repo_root();
    let cfg_path = resolve_config(args.config.clone(), &repo);
    let cfg = config::load(&cfg_path).unwrap_or_else(|e| {
        eprintln!("config: {e} (pass --config PATH)");
        std::process::exit(1);
    });
    let base = cfg.dir.parent().unwrap_or(&cfg.dir).to_path_buf();
    let mut paths = cfg.paths.clone();
    if let Some(p) = &args.overrides.log {
        paths.log = config::resolve(p, &base, &repo);
    }
    if let Some(p) = &args.overrides.stl {
        paths.stl = config::resolve(p, &base, &repo);
    }
    if let Some(p) = &args.overrides.fields {
        paths.fields = config::resolve(p, &base, &repo);
    }
    if let Some(p) = &args.overrides.motion {
        paths.motion = config::resolve(p, &base, &repo);
    }
    if let Some(p) = &args.overrides.socket {
        paths.socket = config::resolve(p, &base, &repo);
    }

    let mut x = x11::X11::connect().unwrap_or_else(|e| {
        eprintln!("X11 connection failed: {e}");
        std::process::exit(1);
    });
    let mask = 1 << 0 | 1 << 1 | 1 << 15 | 1 << 17;
    let win = x.create_window(cfg.w as u16, cfg.h as u16).unwrap_or_else(|e| {
        eprintln!("CreateWindow failed: {e}");
        std::process::exit(1);
    });
    x.select_events(win, mask).unwrap_or_else(|e| {
        eprintln!("select_events failed: {e}");
        std::process::exit(1);
    });
    x.set_title(win, &cfg.title).ok();
    let gc = x.create_gc(win).unwrap_or_else(|e| {
        eprintln!("CreateGC failed: {e}");
        std::process::exit(1);
    });
    x.map_window(win).unwrap_or_else(|e| {
        eprintln!("MapWindow failed: {e}");
        std::process::exit(1);
    });
    x.check_errors("window setup").unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(1);
    });

    let (mut vw, mut vh, mut map_state) = x.window_state(win).unwrap_or((cfg.w as u16, cfg.h as u16, 0));
    for _ in 0..12 {
        if map_state == 2 && vw > 0 && vh > 0 {
            break;
        }
        std::thread::sleep(Duration::from_millis(50));
        if let Ok((w, h, state)) = x.window_state(win) {
            vw = w;
            vh = h;
            map_state = state;
        }
    }
    eprintln!(
        "window 0x{:x} map_state={} size={}x{} title={:?} config={}",
        win,
        map_state,
        vw,
        vh,
        cfg.title,
        cfg_path.display()
    );

    let stl = Stl::load(&paths.stl).unwrap_or_else(|e| {
        eprintln!("STL: {e}");
        Stl::empty()
    });
    let geo = cfg.geometry.clone().unwrap_or_else(|| stl.geometry());

    let mut fields = Vec::new();
    if let Ok(rd) = fs::read_dir(&paths.fields) {
        for ent in rd.flatten() {
            let path = ent.path();
            if path.extension().and_then(|s| s.to_str()) != Some("csv") {
                continue;
            }
            let name = path
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("")
                .to_string();
            if !cfg.fin_states.is_empty() && !cfg.fin_states.iter().any(|s| s.name == name) {
                continue;
            }
            match Field::load(&path, &name) {
                Ok(field) => fields.push(field),
                Err(e) => eprintln!("field {}: {e}", path.display()),
            }
        }
    }
    eprintln!(
        "{}: {} STL triangles, {} recorded fields, socket {}",
        cfg.rocket,
        stl.tris.len(),
        fields.len(),
        paths.socket.display()
    );

    let cd0 = model::neutral_cd(&paths.motion);
    let mut live = LiveField::new(paths.socket.clone(), cfg.label("live").to_string());
    let mut tail = Tail::new(paths.log.clone(), &cfg.gpio_names, args.replay);
    let mut frame = Frame::new(vw as usize, vh as usize, BG);
    let tick = Duration::from_millis(1000 / args.fps as u64);
    let started = Instant::now();
    let mut last_row = Instant::now();
    let mut last_field = String::new();
    let mut last_source = String::new();
    let mut field_change = Instant::now();
    let mut snapshot_done = false;

    loop {
        let now = Instant::now();
        let fresh_rows = tail.poll();
        if !fresh_rows.is_empty() {
            last_row = now;
            if args.print_rows {
                let row = fresh_rows.last().unwrap();
                let gpio: String = row.gpio.iter().map(|b| if *b { '1' } else { '0' }).collect();
                eprintln!(
                    "ROW t={:.0}ms state={} alt={:.2}m gpio={} fin={:?}",
                    row.t_ms, row.state as u64, row.alt_m, gpio, row.fins
                );
            }
        }

        let events = x.poll_events(win);
        if events.key && !args.hold {
            break;
        }
        if let Some((w, h)) = events.resized {
            if w as usize != frame.w || h as usize != frame.h {
                frame = Frame::new(w as usize, h as usize, BG);
            }
        }

        let _ = live.recv();
        let live_frame = if !args.replay && live.fresh(now) {
            live.frame()
        } else {
            None
        };
        let live_field = live.field();
        let source = if args.replay {
            cfg.label("replay")
        } else if live_frame.is_some() {
            cfg.label("live")
        } else if live.frames > 0 {
            cfg.label("stale")
        } else {
            cfg.label("replay")
        };

        let (field, field_name, angles, state_text) = if let Some(frame) = live_frame {
            (
                live_field,
                cfg.label("live").to_string(),
                frame.angles,
                frame.state.clone(),
            )
        } else {
            let closest = cfg
                .fin_states
                .iter()
                .min_by(|a, b| {
                    let da: f64 = a
                        .angles
                        .iter()
                        .zip(tail.row.fins)
                        .map(|(v, f)| (v - f).powi(2))
                        .sum();
                    let db: f64 = b
                        .angles
                        .iter()
                        .zip(tail.row.fins)
                        .map(|(v, f)| (v - f).powi(2))
                        .sum();
                    da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(|state| state.name.clone())
                .unwrap_or_else(|| "CSV".to_string());
            if closest != last_field {
                last_field = closest.clone();
                field_change = now;
            }
            (
                fields.iter().find(|f| f.name == closest),
                closest,
                tail.row.fins,
                format!("{}", tail.row.state as u64),
            )
        };

        if source != last_source {
            eprintln!(
                "source={} field={} seq={} hz={:.1}",
                source,
                field_name,
                live_frame.map(|f| f.seq).unwrap_or(0),
                live.hz
            );
            last_source = source.to_string();
        }

        let tele_live = !args.replay && now.duration_since(last_row) < Duration::from_millis(900);
        render(
            &mut frame,
            &View {
                cfg: &cfg,
                stl: &stl,
                geo: &geo,
                field,
                all_fields: &fields,
                row: &tail.row,
                history: &tail.history,
                cd0,
                field_name: &field_name,
                angles,
                state_text: &state_text,
                source,
                tele_live,
                live_frame,
                live_hz: live.hz,
                now,
                field_change,
            },
        );

        if let Err(e) = x.put_image(win, gc, frame.w, frame.h, &frame.px) {
            eprintln!("PutImage failed: {e}");
            break;
        }
        if !snapshot_done {
            if let Some(path) = &args.snapshot {
                if started.elapsed().as_secs_f64() >= args.snapshot_after {
                    match write_ppm(path, &frame) {
                        Ok(()) => eprintln!("framebuffer snapshot: {}", path.display()),
                        Err(e) => eprintln!("snapshot failed: {e}"),
                    }
                    snapshot_done = true;
                }
            }
        }
        let elapsed = now.elapsed();
        if elapsed < tick {
            std::thread::sleep(tick - elapsed);
        }
    }
}

fn write_ppm(path: &Path, frame: &Frame) -> Result<(), String> {
    let mut fh = File::create(path).map_err(|e| format!("{}: {e}", path.display()))?;
    write!(fh, "P6\n{} {}\n255\n", frame.w, frame.h)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    let mut bytes = Vec::with_capacity(frame.w * 3);
    for pixel in &frame.px {
        bytes.extend_from_slice(&[
            ((pixel >> 16) & 0xff) as u8,
            ((pixel >> 8) & 0xff) as u8,
            (pixel & 0xff) as u8,
        ]);
    }
    fh.write_all(&bytes)
        .map_err(|e| format!("{}: {e}", path.display()))
}

struct View<'a> {
    cfg: &'a Config,
    stl: &'a Stl,
    geo: &'a Geometry,
    field: Option<&'a Field>,
    all_fields: &'a [Field],
    row: &'a TelemetryRow,
    history: &'a VecDeque<TelemetryRow>,
    cd0: Option<f64>,
    field_name: &'a str,
    angles: [f64; 4],
    state_text: &'a str,
    source: &'a str,
    tele_live: bool,
    live_frame: Option<&'a LiveFrame>,
    live_hz: f64,
    now: Instant,
    field_change: Instant,
}

#[allow(clippy::too_many_lines)]
fn render(f: &mut Frame, v: &View) {
    let w = f.w as i64;
    let h = f.h as i64;
    f.clear(BG);
    f.rect(0, 0, w, 48, PANEL);
    f.line(0, 47, w, 47, GRID, 1);
    let status = format!(
        "{}  {}  {}={:.2}S  {}={}",
        v.cfg.rocket,
        v.source,
        v.cfg.label("time"),
        v.row.t_ms / 1000.0,
        v.cfg.label("state"),
        v.state_text
    );
    f.text(&status, 12, 8, WHITE, 2);
    if Frame::text_width(&v.cfg.title, 1) <= w - 24 {
        f.text(&v.cfg.title, 12, 30, DIM, 1);
    }

    let foot_h = 72i64;
    let head = 56i64;
    let panel_h = (h - head - foot_h - 8).max(80);
    let left_x = 10i64;
    let left_w = ((w - 30) * 2 / 3).max(200);
    let right_x = left_x + left_w + 10;
    let right_w = (w - right_x - 10).max(160);

    f.rect(left_x, head, left_w, panel_h, PANEL);
    f.outline(left_x, head, left_w, panel_h, GRID);
    f.text(
        &format!("{} / {} / {}", v.cfg.label("cfd"), v.field_name, v.source),
        left_x + 10,
        head + 8,
        TEXT,
        1,
    );

    let plot_x0 = left_x + 24;
    let plot_x1 = left_x + left_w - 14;
    let plot_y0 = head + 28;
    let plot_y1 = head + panel_h - 52;
    if plot_x1 > plot_x0 && plot_y1 > plot_y0 {
        let span_x = (v.geo.x_max - v.geo.x_min).max(1e-3);
        let span_y = (v.geo.y_max - v.geo.y_min).max(1e-3);
        let scale = ((plot_x1 - plot_x0) as f64 / span_x)
            .min((plot_y1 - plot_y0) as f64 / span_y);
        let mid_y = (plot_y0 + plot_y1) as f64 * 0.5;
        let px = |xm: f64| plot_x0 as f64 + (xm - v.geo.x_min) * scale;
        let py = |ym: f64| mid_y - ym * scale;

        let grid = v.geo.grid_m.max(1e-6);
        let gx = ((v.geo.x_min / grid).ceil() as i64).max(-1000)..=((v.geo.x_max / grid).floor() as i64).min(1000);
        for g in gx {
            let x = px(g as f64 * grid) as i64;
            if x >= plot_x0 && x <= plot_x1 {
                f.line(x, plot_y0, x, plot_y1, GRID, 1);
            }
        }
        let gy = ((v.geo.y_min / grid).ceil() as i64).max(-1000)..=((v.geo.y_max / grid).floor() as i64).min(1000);
        for g in gy {
            let y = py(g as f64 * grid) as i64;
            if y >= plot_y0 && y <= plot_y1 {
                f.line(plot_x0, y, plot_x1, y, GRID, 1);
            }
        }
        let cgx = px(v.geo.cg_x) as i64;
        f.dashed_line(cgx, plot_y0, cgx, plot_y1, DIM, 6, 6, 0, 1);

        if let Some(field) = v.field {
            let lines = field.streamlines();
            let mut vmax = 0f32;
            for line in &lines {
                for point in line {
                    vmax = vmax.max(point.2);
                }
            }
            let vmax = vmax.max(1e-5);
            let phase = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis() as i64
                / 22;
            for (i, line) in lines.iter().enumerate() {
                for pair in line.windows(2) {
                    let speed = ((pair[0].2 + pair[1].2) * 0.5 / vmax).clamp(0.0, 1.0);
                    let shade = shade(WHITE, 0.3 + 0.6 * speed as f64);
                    f.dashed_line(
                        px(pair[0].0 as f64) as i64,
                        py(pair[0].1 as f64) as i64,
                        px(pair[1].0 as f64) as i64,
                        py(pair[1].1 as f64) as i64,
                        shade,
                        10,
                        8,
                        phase + i as i64 * 7,
                        1,
                    );
                }
            }
            let mut uy = 0f32;
            let mut count = 0;
            if field.nx > 14 {
                for xi in field.nx - 12..field.nx - 2 {
                    for yi in 0..field.ny {
                        uy += field.u[xi * field.ny + yi][1];
                        count += 1;
                    }
                }
            }
            let uy = uy / count.max(1) as f32;
            let ax = plot_x1 - 14;
            let ay = mid_y as i64;
            let by = ay - (uy * 9000.0).clamp(-40.0, 40.0) as i64;
            f.line(ax, ay, ax, by, TEXT, 2);
            f.line(ax - 4, by + if by < ay { 6 } else { -6 }, ax, by, TEXT, 1);
            f.line(ax + 4, by + if by < ay { 6 } else { -6 }, ax, by, TEXT, 1);
        } else {
            f.text(v.cfg.label("empty_cfd"), left_x + 40, head + 120, DIM, 2);
        }

        draw_rocket(f, v.stl, v.geo, v.row, v.angles, &px, &py);
    }

    let cd0 = v
        .cd0
        .map(|c| format!("{c:.5}"))
        .unwrap_or_else(|| "NA".to_string());
    let mut info = format!(
        "{} {} {}x{} DX={:.4} CD0={} AGE={}MS ANG={:.0}/{:.0}/{:.0}/{:.0}",
        v.cfg.label("field"),
        v.field_name,
        v.field.map(|x| x.nx).unwrap_or(0),
        v.field.map(|x| x.ny).unwrap_or(0),
        v.field.map(|x| x.dx).unwrap_or(0.0),
        cd0,
        v.now.duration_since(v.field_change).as_millis(),
        v.angles[0],
        v.angles[1],
        v.angles[2],
        v.angles[3]
    );
    if let Some(frame) = v.live_frame {
        info.push_str(&format!(" T={:.0}MS", frame.t_ms));
        if !frame.forces.is_empty() {
            let forces = frame
                .forces
                .iter()
                .map(|(k, value)| format!("{k}={value:.2}"))
                .collect::<Vec<_>>()
                .join(" ");
            info.push_str(&format!(" {} {}", v.cfg.label("forces"), forces));
        }
    }
    f.text(
        &fit(&info, ((left_w - 20) / 6).max(8)),
        left_x + 10,
        head + panel_h - 20,
        DIM,
        1,
    );

    f.rect(right_x, head, right_w, panel_h, PANEL);
    f.outline(right_x, head, right_w, panel_h, GRID);
    f.text(v.cfg.label("telemetry"), right_x + 10, head + 8, WHITE, 2);
    f.circle(
        right_x + right_w - 14,
        head + 13,
        4,
        if v.tele_live { WHITE } else { AMBER },
    );

    let rows = [
        (
            v.cfg.label("time"),
            format!("{:.2} S", v.row.t_ms / 1000.0),
        ),
        (v.cfg.label("alt"), format!("{:.2} M", v.row.alt_m)),
        (v.cfg.label("vv"), format!("{:.2} M/S", v.row.vv_ms)),
        (
            v.cfg.label("pred_apo"),
            format!("{:.2} M", v.row.pred_apo_m),
        ),
        (
            v.cfg.label("pitch"),
            format!("{:.2} DEG", v.row.pitch_deg),
        ),
        (v.cfg.label("roll"), format!("{:.2} DEG", v.row.roll_deg)),
        (v.cfg.label("fault"), format!("{:.0}", v.row.fault_code)),
    ];
    let value_x = right_x + 130;
    let value_chars = ((right_w - 140) / 6).max(8);
    let mut y = head + 30;
    for (name, value) in rows {
        let color = if name == v.cfg.label("fault") && v.row.fault_code > 0.5 {
            AMBER
        } else {
            TEXT
        };
        f.text(name, right_x + 10, y, DIM, 1);
        f.text(&fit(&value, value_chars), value_x, y, color, 1);
        y += 16;
    }
    y += 4;
    f.text(
        &format!("{}={}", v.cfg.label("state"), v.state_text),
        right_x + 10,
        y,
        TEXT,
        1,
    );
    y += 18;

    f.text(
        &format!("{} 0-3", v.cfg.label("fin_act")),
        right_x + 10,
        y,
        DIM,
        1,
    );
    y += 14;
    for i in 0..4 {
        let angle = v.angles[i].clamp(-15.0, 15.0);
        let mid = right_x + 140;
        f.text(
            &format!("{} {}", v.cfg.label("fin_act"), i),
            right_x + 10,
            y,
            DIM,
            1,
        );
        f.line(mid, y + 3, mid + angle as i64 * 5, y + 3, WHITE, 2);
        f.line(mid, y, mid, y + 7, DIM, 1);
        f.text(
            &format!("{:+.1}", angle),
            right_x + right_w - 68,
            y,
            TEXT,
            1,
        );
        y += 13;
    }
    for i in 0..4 {
        f.text(
            &format!("{} {}", v.cfg.label("pwm"), i),
            right_x + 10,
            y,
            DIM,
            1,
        );
        f.text(
            &format!("{:.0} US", v.row.pwm_us[i]),
            value_x,
            y,
            TEXT,
            1,
        );
        y += 13;
    }

    y += 4;
    let bits: String = v.row.gpio.iter().map(|b| if *b { '1' } else { '0' }).collect();
    f.text(
        &format!("{} {}", v.cfg.label("gpio"), bits),
        right_x + 10,
        y,
        TEXT,
        1,
    );
    y += 16;
    let columns = v.cfg.gpio_names.len().div_ceil(2).max(1);
    let column_w = (right_w - 20) / 2;
    for (i, name) in v.cfg.gpio_names.iter().enumerate() {
        let yy = y + (i % columns) as i64 * 13;
        if yy + 7 > head + panel_h - 6 {
            break;
        }
        let xx = right_x + 10 + (i / columns) as i64 * column_w;
        let on = v.row.gpio.get(i).copied().unwrap_or(false);
        f.circle(xx + 4, yy + 3, 3, if on { WHITE } else { GRID });
        f.text(
            &fit(&name.to_ascii_uppercase(), (column_w / 6).saturating_sub(8)),
            xx + 12,
            yy,
            if on { TEXT } else { DIM },
            1,
        );
        f.text(
            if on { "1" } else { "0" },
            xx + column_w - 12,
            yy,
            if on { WHITE } else { DIM },
            1,
        );
    }

    let feed_y = h - foot_h;
    f.rect(0, feed_y, w, foot_h, PANEL);
    f.line(0, feed_y, w, feed_y, GRID, 1);
    f.text(v.cfg.label("feed"), 10, feed_y + 8, DIM, 1);
    let max_rows = ((w - 100) / 6 / 20).clamp(1, 8) as usize;
    let start = v.history.len().saturating_sub(max_rows);
    let feed = v
        .history
        .iter()
        .skip(start)
        .map(|row| {
            format!(
                "{:.1}/{:.1}/{:.1}/{}",
                row.t_ms / 1000.0,
                row.alt_m,
                row.vv_ms,
                row.state as u64
            )
        })
        .collect::<Vec<_>>()
        .join(" ");
    f.text(
        &fit(&feed, ((w - 90) / 6).max(8)),
        84,
        feed_y + 8,
        TEXT,
        1,
    );
    let seq = v.live_frame.map(|frame| frame.seq).unwrap_or(0);
    let footer = format!(
        "{} {} {} {:.1} {} {} {} {} (ROWS {})",
        v.cfg.label("seq"),
        seq,
        v.cfg.label("hz"),
        v.live_hz,
        v.cfg.label("field"),
        v.field_name,
        v.source,
        v.cfg.label("quit"),
        v.all_fields.len()
    );
    f.text(
        &fit(&footer, ((w - 20) / 6).max(8)),
        10,
        feed_y + 28,
        DIM,
        1,
    );
}

fn fit(text: &str, max_chars: i64) -> String {
    let limit = max_chars.max(0) as usize;
    if text.len() <= limit {
        return text.to_string();
    }
    if limit <= 3 {
        return ".".repeat(limit);
    }
    format!("{}...", &text[..limit - 3])
}

fn draw_rocket<F: Fn(f64) -> f64, G: Fn(f64) -> f64>(
    f: &mut Frame,
    stl: &Stl,
    geo: &Geometry,
    row: &TelemetryRow,
    angles: [f64; 4],
    px: F,
    py: G,
) {
    if stl.tris.is_empty() {
        return;
    }
    let pitch = (row.pitch_deg as f64).to_radians();
    let roll = (row.roll_deg as f64).to_radians();
    let mut projected = Vec::with_capacity(stl.tris.len());
    for (normal, a, b, c) in &stl.tris {
        let va = transform(a, geo, angles, pitch, roll);
        let vb = transform(b, geo, angles, pitch, roll);
        let vc = transform(c, geo, angles, pitch, roll);
        let depth = (va[2] + vb[2] + vc[2]) / 3.0;
        projected.push((
            va,
            vb,
            vc,
            [normal[0] as f64, normal[1] as f64, normal[2] as f64],
            depth,
        ));
    }
    projected.sort_by(|a, b| a.4.partial_cmp(&b.4).unwrap_or(std::cmp::Ordering::Equal));
    for (a, b, c, normal, _) in projected {
        let light = (normal[0] * -0.35 + normal[1] * -0.45 + normal[2] * 0.82).max(0.0);
        let color = shade(WHITE, 0.14 + 0.34 * light);
        f.tri(
            (px(a[0]) as i64, py(a[1]) as i64),
            (px(b[0]) as i64, py(b[1]) as i64),
            (px(c[0]) as i64, py(c[1]) as i64),
            color,
        );
    }
    f.circle(px(geo.cg_x) as i64, py(0.0) as i64, 2, shade(WHITE, 0.85));
    if geo.hinge_r > 0.0 {
        for i in 0..4 {
            let ang = i as f64 * std::f64::consts::FRAC_PI_2;
            let hy = geo.hinge_r * ang.cos();
            let hz = geo.hinge_r * ang.sin();
            let y = hy * roll.cos() - hz * roll.sin();
            f.circle(
                px(geo.hinge_x) as i64,
                py(y) as i64,
                1,
                shade(WHITE, 0.65),
            );
        }
    }
}

fn transform(
    p: &[f32; 3],
    geo: &Geometry,
    angles: [f64; 4],
    pitch: f64,
    roll: f64,
) -> [f64; 3] {
    let mut x = p[0] as f64;
    let mut y = p[1] as f64;
    let mut z = p[2] as f64;
    let radial = (y * y + z * z).sqrt();
    if x > geo.hinge_x + 0.005 && radial > geo.hinge_r && geo.hinge_r > 0.0 {
        let theta = z.atan2(y);
        let idx = ((theta / std::f64::consts::FRAC_PI_2).round() as i64).rem_euclid(4) as usize;
        let a = angles[idx].to_radians();
        let dx = x - geo.hinge_x;
        let dr = radial - geo.hinge_r;
        let nx = dx * a.cos() - dr * a.sin();
        let nr = dx * a.sin() + dr * a.cos();
        x = geo.hinge_x + nx;
        let radius = geo.hinge_r + nr;
        y = radius * theta.cos();
        z = radius * theta.sin();
    }
    let dx = x - geo.cg_x;
    let xp = dx * pitch.cos() - y * pitch.sin();
    let yp = dx * pitch.sin() + y * pitch.cos();
    x = xp + geo.cg_x;
    y = yp;
    let yr = y * roll.cos() - z * roll.sin();
    let zr = y * roll.sin() + z * roll.cos();
    [x, yr, zr]
}
