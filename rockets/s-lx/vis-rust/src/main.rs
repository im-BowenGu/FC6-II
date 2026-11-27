//! S-LX Xwayland visualiser: real HIL GPIO + real Taichi CFD velocity slices.
//!
//! Pure std Rust. The window is a real X11 client on Xwayland; frames are
//! rasterised in software and uploaded with ZPixmap PutImage.

mod draw;
mod model;
mod x11;

use draw::{rgb, shade, Frame};
use model::{neutral_cd, Field, Stl, Tail, GPIO_NAMES, FIN_STATES};
use std::env;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const W: usize = 1240;
const H: usize = 760;

const BG: u32 = rgb(7, 11, 18);
const PANEL: u32 = rgb(13, 21, 33);
const GRID: u32 = rgb(26, 40, 58);
const TEXT: u32 = rgb(214, 228, 244);
const DIM: u32 = rgb(112, 134, 160);
const ACCENT: u32 = rgb(84, 208, 255);
const GREEN: u32 = rgb(82, 230, 145);
const RED: u32 = rgb(255, 96, 108);
const AMBER: u32 = rgb(255, 188, 74);
const STEEL: u32 = rgb(151, 171, 192);
const WHITE: u32 = rgb(245, 248, 252);

struct Args {
    log: PathBuf,
    stl: PathBuf,
    fields: PathBuf,
    motion: PathBuf,
    replay: bool,
    fps: u32,
    /// Dump the exact uploaded framebuffer as a P6 PPM after `--snapshot-after` seconds.
    /// This is the same buffer PutImage sends to Xwayland, so it is an honest
    /// capture of the real window content without needing an X screenshot tool.
    snapshot: Option<PathBuf>,
    snapshot_after: f64,
    print_rows: bool,
    /// Keep the window open even when a key is pressed (useful for HIL runs).
    hold: bool,
}

fn default_args() -> Args {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
    Args {
        log: root.join("hil/logs/bridge.csv"),
        stl: root.join("cfd/fluidx3d/S-LX_cfd.stl"),
        fields: root.join("build/cfd/fields"),
        motion: root.join("build/cfd/motion.json"),
        replay: false,
        fps: 30,
        snapshot: None,
        snapshot_after: 1.0,
        print_rows: false,
        hold: false,
    }
}

fn parse_args() -> Args {
    let mut a = default_args();
    let mut it = env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--replay" => a.replay = true,
            "--log" => a.log = PathBuf::from(it.next().unwrap_or_default()),
            "--stl" => a.stl = PathBuf::from(it.next().unwrap_or_default()),
            "--fields" => a.fields = PathBuf::from(it.next().unwrap_or_default()),
            "--motion" => a.motion = PathBuf::from(it.next().unwrap_or_default()),
            "--fps" => {
                a.fps = it.next().and_then(|v| v.parse().ok()).unwrap_or(30).clamp(1, 120)
            }
            "--snapshot" => a.snapshot = Some(PathBuf::from(it.next().unwrap_or_default())),
            "--snapshot-after" => {
                let secs: f64 = it.next().and_then(|v| v.parse().ok()).unwrap_or(1.0);
                a.snapshot_after = secs.max(0.0);
            }
            "--print-rows" => a.print_rows = true,
            "--hold" => a.hold = true,
            "-h" | "--help" => {
                println!(
                    "slx-cfd-vis [--replay] [--log PATH] [--stl PATH] \
                     [--fields DIR] [--motion PATH] [--fps N]\n\
                     [ --snapshot PPM [--snapshot-after SECS] ] [--print-rows] [--hold]"
                );
                std::process::exit(0);
            }
            _ => eprintln!("unknown argument: {arg}"),
        }
    }
    a
}

fn main() {
    let args = parse_args();
    let mut x = x11::X11::connect().unwrap_or_else(|e| {
        eprintln!("X11 connection failed: {e}");
        std::process::exit(1);
    });
    let mask = 1 << 0 | 1 << 1 | 1 << 15 | 1 << 17; // key/expose/structure
    let win = x.create_window(W as u16, H as u16).unwrap_or_else(|e| {
        eprintln!("CreateWindow failed: {e}");
        std::process::exit(1);
    });
    x.select_events(win, mask).unwrap_or_else(|e| {
        eprintln!("select_events failed: {e}");
        std::process::exit(1);
    });
    x.set_title(win, "S-LX CFD / HIL - Xwayland").ok();
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

    let stl = Stl::load(&args.stl).unwrap_or_else(|e| {
        eprintln!("STL: {e}");
        std::process::exit(1);
    });
    let mut fields: Vec<Field> = Vec::new();
    if let Ok(rd) = std::fs::read_dir(&args.fields) {
        for ent in rd.flatten() {
            let p = ent.path();
            if p.extension().and_then(|s| s.to_str()) != Some("csv") {
                continue;
            }
            let name = p.file_stem().and_then(|s| s.to_str()).unwrap_or("").to_string();
            if !FIN_STATES.iter().any(|(n, _)| *n == name) {
                continue;
            }
            match Field::load(&p, &name) {
                Ok(f) => fields.push(f),
                Err(e) => eprintln!("field {}: {e}", p.display()),
            }
        }
    }
    if fields.is_empty() {
        eprintln!(
            "no Taichi velocity slices in {} (run make cfd-record CFD_ARGS=--dump-field)",
            args.fields.display()
        );
    }
    let cd0 = neutral_cd(&args.motion);
    let mut tail = Tail::new(args.log.clone(), args.replay);
    let mut frame = Frame::new(W, H, BG);
    let mut last_row = Instant::now();
    let mut field_name = String::from("neutral");
    let mut last_field_change = Instant::now();
    let tick = Duration::from_millis(1000 / args.fps as u64);

    eprintln!(
        "S-LX visualiser: X window 0x{:x} {}x{} on DISPLAY={:?}, STL {} tris, {} CFD slices, log {} ({})",
        win, W, H,
        env::var("DISPLAY").unwrap_or_else(|_| ":0".into()),
        stl.tris.len(),
        fields.len(),
        args.log.display(),
        if args.replay { "replay" } else { "live tail" }
    );
    eprintln!("press any key in the window to quit");

    let started = Instant::now();
    let mut snapshot_done = false;
    loop {
        let now = Instant::now();
        let new_rows = tail.poll();
        if new_rows > 0 {
            last_row = now;
        }
        let row = tail.row.clone();
        if args.print_rows && new_rows > 0 {
            let gpio: String = row.gpio.iter().map(|b| if *b { '1' } else { '0' }).collect();
            eprintln!(
                "ROW t={:.0}ms state={} alt={:.2}m gpio={} fin={:?}",
                row.t_ms, row.state as u32, row.alt_m, gpio, row.fins
            );
        }

        // Pick the recorded CFD slice closest to the actual fin angles.
        if let Some((best, _)) = FIN_STATES
            .iter()
            .min_by(|(_, a), (_, b)| {
                let da: f64 = a.iter().zip(row.fins).map(|(v, f)| (v - f).powi(2)).sum();
                let db: f64 = b.iter().zip(row.fins).map(|(v, f)| (v - f).powi(2)).sum();
                da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
            })
        {
            if best.to_string() != field_name {
                field_name = best.to_string();
                last_field_change = now;
            }
        }
        let field = fields.iter().find(|f| f.name == field_name);

        render(
            &mut frame,
            &stl,
            field,
            &fields,
            &row,
            cd0,
            &field_name,
            &args.log,
            args.replay,
            now,
            last_row,
            tail.updates,
            last_field_change,
        );
        if let Err(e) = x.put_image(win, gc, W, H, &frame.px) {
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
        let key_pressed = x.poll_key(); // always drain events/errors
        if !args.hold && key_pressed {
            break;
        }
        let elapsed = now.elapsed();
        if elapsed < tick {
            std::thread::sleep(tick - elapsed);
        }
    }
}

/// Write a binary P6 PPM of the exact framebuffer uploaded with PutImage.
/// PPM is trivial to decode with `convert frame.ppm frame.png` (no X needed).
fn write_ppm(path: &Path, frame: &Frame) -> Result<(), String> {
    use std::io::Write;
    let mut fh = std::fs::File::create(path)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    write!(fh, "P6\n{} {}\n255\n", frame.w, frame.h)
        .map_err(|e| format!("{}: {e}", path.display()))?;
    let mut row = Vec::with_capacity(frame.w * 3);
    for pixel in &frame.px {
        row.extend_from_slice(&[
            ((pixel >> 16) & 0xff) as u8,
            ((pixel >> 8) & 0xff) as u8,
            (pixel & 0xff) as u8,
        ]);
    }
    fh.write_all(&row).map_err(|e| format!("{}: {e}", path.display()))
}

#[allow(clippy::too_many_arguments)]
fn render(
    f: &mut Frame,
    stl: &Stl,
    field: Option<&Field>,
    all_fields: &[Field],
    row: &model::TelemetryRow,
    cd0: f64,
    field_name: &str,
    log_path: &Path,
    replay: bool,
    now: Instant,
    last_row: Instant,
    updates: u64,
    field_change: Instant,
) {
    f.clear(BG);
    f.rect(0, 0, W as i64, 56, PANEL);
    f.rect(0, 55, W as i64, 2, ACCENT);
    f.text("S-LX CFD / HIL VISUALIZER", 18, 12, WHITE, 3);
    f.text("REAL XWAYLAND WINDOW - RUST X11 CLIENT", 18, 38, DIM, 1);
    let stamp = format!("XWAYLAND :0  T+{:.2}S", row.t_ms / 1000.0);
    f.text(&stamp, W as i64 - Frame::text_width(&stamp, 2) - 18, 20, ACCENT, 2);

    // ---- CFD / STL panel -------------------------------------------------
    f.rect(16, 70, 828, 500, PANEL);
    f.outline(16, 70, 828, 500, GRID);
    f.text("TAICHI D3Q19 FLOW + ROCKET STL", 30, 82, TEXT, 2);
    let ox = 44.0f64;
    let cy = 300.0f64;
    let scale = 520.0f64;
    let px = |xm: f32| ox + xm as f64 * scale;
    let py = |ym: f32| cy - ym as f64 * scale;

    for gx in 0..=14 {
        let x = px(gx as f32 * 0.1);
        f.line(x as i64, 108, x as i64, 430, GRID, 1);
    }
    for gy in -2..=2 {
        let y = py(gy as f32 * 0.1);
        f.line(44, y as i64, 778, y as i64, GRID, 1);
    }
    f.dashed_line(px(0.514) as i64, 108, px(0.514) as i64, 430, AMBER, 5, 5, 0, 1);
    f.text("CG", px(0.514) as i64 + 4, 110, AMBER, 1);

    // Real velocity field: animated dashes along integrated streamlines.
    if let Some(fld) = field {
        let lines = fld.streamlines();
        let mut vmax = 0.0f32;
        for l in &lines {
            for p in l {
                vmax = vmax.max(p.2);
            }
        }
        let vmax = vmax.max(1e-5);
        let phase = (now.elapsed().as_millis() / 22) as i64;
        for (i, line) in lines.iter().enumerate() {
            for w in line.windows(2) {
                let speed = (w[0].2 + w[1].2) * 0.5 / vmax;
                let c = if speed > 0.72 {
                    rgb(255, 200, 90)
                } else if speed > 0.45 {
                    rgb(120, 240, 190)
                } else {
                    ACCENT
                };
                f.dashed_line(
                    px(w[0].0) as i64,
                    py(w[0].1) as i64,
                    px(w[1].0) as i64,
                    py(w[1].1) as i64,
                    c,
                    12,
                    9,
                    phase + i as i64 * 7,
                    2,
                );
            }
        }
        let mut uy = 0.0f32;
        let mut n = 0;
        if fld.nx > 14 {
            for xi in fld.nx - 12..fld.nx - 2 {
                for yi in 0..fld.ny {
                    uy += fld.u[xi * fld.ny + yi][1];
                    n += 1;
                }
            }
        }
        let uy = uy / n.max(1) as f32;
        let ax = px(1.02) as i64;
        let ay = py(0.0) as i64;
        let by = ay - (uy * 9000.0).clamp(-55.0, 55.0) as i64;
        f.line(ax, ay, ax, by, AMBER, 3);
        f.line(ax - 5, by + if by < ay { 8 } else { -8 }, ax, by, AMBER, 2);
        f.line(ax + 5, by + if by < ay { 8 } else { -8 }, ax, by, AMBER, 2);
    } else {
        f.text("NO CFD SLICE - RUN cfd-record WITH --dump-field", 60, 210, RED, 2);
    }

    draw_rocket(f, stl, row, px, py);

    f.text("FLOW", 30, 446, ACCENT, 2);
    f.line(84, 450, 130, 450, ACCENT, 2);
    f.text("SLOW", 140, 444, DIM, 1);
    f.line(186, 450, 232, 450, rgb(120, 240, 190), 2);
    f.text("FAST", 242, 444, DIM, 1);
    f.line(288, 450, 334, 450, rgb(255, 200, 90), 2);
    f.text("STL", 370, 444, STEEL, 1);
    f.rect(404, 438, 16, 12, STEEL);
    let label = format!(
        "FIELD {} / {} SLICES / CD0 {:.5}",
        field_name.to_uppercase(),
        all_fields.len(),
        cd0
    );
    f.text(&label, 470, 444, TEXT, 1);
    let selftest = format!(
        "ACT {} {} {} {}   CMD-PWM {:.0} {:.0} {:.0} {:.0} US",
        row.fins[0], row.fins[1], row.fins[2], row.fins[3],
        row.pwm_us[0], row.pwm_us[1], row.pwm_us[2], row.pwm_us[3]
    );
    f.text(&selftest, 30, 478, DIM, 1);
    let field_age = now.duration_since(field_change).as_millis();
    let fit = format!(
        "FIELD SWITCH AGE {} MS   STATE {}   ALT {:.2} M   VV {:.2} M/S",
        field_age, row.state as u32, row.alt_m, row.vv_ms
    );
    f.text(&fit, 30, 500, DIM, 1);
    f.text("NOSE X=0", 46, 552, DIM, 1);
    f.text("FLOW +X", 700, 552, DIM, 1);
    f.text("FINS X=0.83M", 600, 70, DIM, 1);

    // ---- GPIO panel ------------------------------------------------------
    f.rect(860, 70, 364, 500, PANEL);
    f.outline(860, 70, 364, 500, GRID);
    f.text("REAL-TIME GPIO", 874, 82, WHITE, 2);
    let live = !replay && now.duration_since(last_row) < Duration::from_millis(900);
    let status = if replay {
        "REPLAY"
    } else if live {
        "LIVE"
    } else {
        "STALE"
    };
    let sc = if live || replay { GREEN } else { RED };
    f.circle(1182, 88, 5, sc);
    f.text(status, 1140, 82, sc, 1);

    for (i, name) in GPIO_NAMES.iter().enumerate() {
        let col = i / 7;
        let r = i % 7;
        let x = 876i64 + col as i64 * 178;
        let y = 118i64 + r as i64 * 30;
        let on = row.gpio[i];
        let c = if on { GREEN } else { rgb(45, 58, 74) };
        f.circle(x + 8, y + 7, 6, c);
        if on {
            f.circle(x + 8, y + 7, 2, WHITE);
        }
        f.text(&name.to_uppercase(), x + 22, y + 2, if on { WHITE } else { DIM }, 1);
        let txt = if on { "ON" } else { "OFF" };
        f.text(txt, x + 116, y + 2, c, 1);
    }

    f.line(874, 344, 1210, 344, GRID, 1);
    f.text("FLIGHT STATE", 876, 356, ACCENT, 2);
    let state_name = ["INIT", "ARM", "BURN", "COAST", "APOGEE", "LAND", "FAULT"];
    let sn = state_name.get(row.state as usize).copied().unwrap_or("UNKNOWN");
    f.text(sn, 1080, 356, WHITE, 2);

    let lines = [
        format!("TIME      {:8.2} S", row.t_ms / 1000.0),
        format!("ALTITUDE  {:8.2} M", row.alt_m),
        format!("PRED APO  {:8.2} M", row.pred_apo_m),
        format!("PITCH     {:8.2} DEG", row.pitch_deg),
        format!("ROLL      {:8.2} DEG", row.roll_deg),
        format!("FAULT     {:8.0}", row.fault_code),
    ];
    for (i, l) in lines.iter().enumerate() {
        f.text(l, 876, 382 + i as i64 * 18, TEXT, 1);
    }
    f.text("FIN ACTUATION (DEG)", 876, 500, ACCENT, 1);
    for i in 0..4 {
        let y = 518 + i as i64 * 11;
        f.text(&format!("F{i}"), 876, y, DIM, 1);
        let v = row.fins[i].clamp(-15.0, 15.0);
        let mid = 1010i64;
        f.line(mid, y + 3, mid + (v as i64 * 5), y + 3, ACCENT, 3);
        f.line(mid, y, mid, y + 7, DIM, 1);
    }

    // ---- footer ----------------------------------------------------------
    f.rect(0, 580, W as i64, H as i64 - 580, PANEL);
    f.rect(0, 580, W as i64, 2, ACCENT);
    f.text("SOURCE", 18, 594, ACCENT, 2);
    f.text(&format!("GPIO {}", log_path.display()), 110, 594, TEXT, 1);
    f.text(&format!("ROWS PARSED {}", updates), 18, 624, DIM, 1);
    f.text(
        "GPIO IS TAILED LIVE FROM hil/logs/bridge.csv (VERILATED STM32 PERIPHERALS); PRESS ANY KEY TO QUIT",
        260,
        624,
        DIM,
        1,
    );
    f.text(
        "FLOW LINES ARE INTEGRATED THROUGH THE RECORDED TAICHI/CUDA D3Q19 VELOCITY SLICE; STL IS S-LX CFD GEOMETRY",
        18,
        650,
        DIM,
        1,
    );
    f.text(
        "REDUCED-ORDER VIEW: STEADY SLICE + HIL TELEMETRY, NOT A LIVE 100 HZ NAVIER-STOKES SOLVE",
        18,
        676,
        AMBER,
        1,
    );
}

#[allow(dead_code)]
fn draw_rocket<F: Fn(f32) -> f64, G: Fn(f32) -> f64>(
    f: &mut Frame,
    stl: &Stl,
    row: &model::TelemetryRow,
    px: F,
    py: G,
) {
    let pitch = (row.pitch_deg as f64).to_radians();
    let roll = (row.roll_deg as f64).to_radians();
    let mut projected: Vec<([f64; 3], [f64; 3], [f64; 3], [f64; 3], f64)> =
        Vec::with_capacity(stl.tris.len());
    for (n, a, b, c) in &stl.tris {
        let va = transform(a, pitch, roll, &row.fins);
        let vb = transform(b, pitch, roll, &row.fins);
        let vc = transform(c, pitch, roll, &row.fins);
        let depth = (va[2] + vb[2] + vc[2]) / 3.0;
        let normal = [n[0] as f64, n[1] as f64, n[2] as f64];
        projected.push((va, vb, vc, normal, depth));
    }
    projected.sort_by(|a, b| a.4.partial_cmp(&b.4).unwrap_or(std::cmp::Ordering::Equal));

    for (a, b, c, n, _) in projected {
        let ax = px(a[0] as f32);
        let ay = py(a[1] as f32);
        let bx = px(b[0] as f32);
        let by = py(b[1] as f32);
        let cx = px(c[0] as f32);
        let cy = py(c[1] as f32);
        let lx = -0.35f64;
        let ly = -0.45f64;
        let lz = 0.82f64;
        let k = (n[0] as f64 * lx + n[1] as f64 * ly + n[2] as f64 * lz).max(0.0);
        let color = shade(STEEL, 0.42 + 0.72 * k);
        f.tri(
            (ax as i64, ay as i64),
            (bx as i64, by as i64),
            (cx as i64, cy as i64),
            color,
        );
    }
    // Nose and hinge markers.
    f.circle(px(0.0) as i64, py(0.0) as i64, 3, AMBER);
    for i in 0..4 {
        let ang = i as f64 * std::f64::consts::FRAC_PI_2;
        let hy = 0.038 * ang.cos();
        let hz = 0.038 * ang.sin();
        let y2 = hy * roll.cos() - hz * roll.sin();
        f.circle(px(0.81) as i64, py(y2 as f32) as i64, 2, ACCENT);
    }
}

fn transform(
    p: &[f32; 3],
    pitch: f64,
    roll: f64,
    fins: &[f64; 4],
) -> [f64; 3] {
    let mut x = p[0] as f64;
    let mut y = p[1] as f64;
    let mut z = p[2] as f64;
    let radial = (y * y + z * z).sqrt();
    if x > 0.805 && radial > 0.045 {
        let theta = z.atan2(y);
        let idx = ((theta / std::f64::consts::FRAC_PI_2).round() as i64).rem_euclid(4) as usize;
        let a = fins[idx].to_radians();
        let dx = x - 0.80;
        let dr = radial - 0.038;
        let nx = dx * a.cos() - dr * a.sin();
        let nr = dx * a.sin() + dr * a.cos();
        x = 0.80 + nx;
        let r = 0.038 + nr;
        y = r * theta.cos();
        z = r * theta.sin();
    }
    let dx = x - 0.514;
    let xp = dx * pitch.cos() - y * pitch.sin();
    let yp = dx * pitch.sin() + y * pitch.cos();
    x = xp + 0.514;
    y = yp;
    let yr = y * roll.cos() - z * roll.sin();
    let zr = y * roll.sin() + z * roll.cos();
    [x, yr, zr]
}
