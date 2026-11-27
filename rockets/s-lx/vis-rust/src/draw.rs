//! Tiny CPU rasteriser: solid shapes, scanline triangles and a 5x7 font.

pub const fn rgb(r: u8, g: u8, b: u8) -> u32 {
    ((r as u32) << 16) | ((g as u32) << 8) | b as u32
}

pub struct Frame {
    pub w: usize,
    pub h: usize,
    pub px: Vec<u32>,
}

impl Frame {
    pub fn new(w: usize, h: usize, bg: u32) -> Self {
        Self { w, h, px: vec![bg; w * h] }
    }

    pub fn clear(&mut self, c: u32) {
        self.px.fill(c);
    }

    #[inline]
    pub fn set(&mut self, x: i64, y: i64, c: u32) {
        if x >= 0 && y >= 0 && (x as usize) < self.w && (y as usize) < self.h {
            self.px[y as usize * self.w + x as usize] = c;
        }
    }

    pub fn rect(&mut self, x: i64, y: i64, w: i64, h: i64, c: u32) {
        for yy in y..y + h {
            for xx in x..x + w {
                self.set(xx, yy, c);
            }
        }
    }

    pub fn outline(&mut self, x: i64, y: i64, w: i64, h: i64, c: u32) {
        for xx in x..x + w {
            self.set(xx, y, c);
            self.set(xx, y + h - 1, c);
        }
        for yy in y..y + h {
            self.set(x, yy, c);
            self.set(x + w - 1, yy, c);
        }
    }

    pub fn line(&mut self, x0: i64, y0: i64, x1: i64, y1: i64, c: u32, thick: i64) {
        let mut x = x0;
        let mut y = y0;
        let dx = (x1 - x0).abs();
        let dy = -(y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;
        loop {
            let half = thick / 2;
            self.rect(x - half, y - half, thick.max(1), thick.max(1), c);
            if x == x1 && y == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }
            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
    }

    pub fn dashed_line(
        &mut self,
        x0: i64,
        y0: i64,
        x1: i64,
        y1: i64,
        c: u32,
        dash: i64,
        gap: i64,
        phase: i64,
        thick: i64,
    ) {
        let dx = x1 - x0;
        let dy = y1 - y0;
        let len = ((dx * dx + dy * dy) as f64).sqrt();
        if len < 1.0 {
            return;
        }
        let len_i = len as i64;
        let period = dash + gap;
        let mut s = ((phase % period) + period) % period;
        while s < len_i {
            let e = (s + dash).min(len_i);
            let ax = x0 + ((dx as f64 * s as f64 / len) as i64);
            let ay = y0 + ((dy as f64 * s as f64 / len) as i64);
            let bx = x0 + ((dx as f64 * e as f64 / len) as i64);
            let by = y0 + ((dy as f64 * e as f64 / len) as i64);
            self.line(ax, ay, bx, by, c, thick);
            s += period;
        }
    }

    pub fn circle(&mut self, cx: i64, cy: i64, r: i64, c: u32) {
        for y in -r..=r {
            for x in -r..=r {
                if x * x + y * y <= r * r {
                    self.set(cx + x, cy + y, c);
                }
            }
        }
    }

    /// Scanline fill of a flat-shaded triangle using integer edge functions.
    pub fn tri(
        &mut self,
        a: (i64, i64),
        b: (i64, i64),
        c: (i64, i64),
        color: u32,
    ) {
        let area = edge(a.0, a.1, b.0, b.1, c.0, c.1);
        if area == 0 {
            self.line(a.0, a.1, b.0, b.1, color, 1);
            self.line(b.0, b.1, c.0, c.1, color, 1);
            self.line(c.0, c.1, a.0, a.1, color, 1);
            return;
        }
        let minx = a.0.min(b.0).min(c.0).max(0);
        let maxx = a.0.max(b.0).max(c.0).min(self.w as i64 - 1);
        let miny = a.1.min(b.1).min(c.1).max(0);
        let maxy = a.1.max(b.1).max(c.1).min(self.h as i64 - 1);
        for y in miny..=maxy {
            for x in minx..=maxx {
                let w0 = edge(b.0, b.1, c.0, c.1, x, y);
                let w1 = edge(c.0, c.1, a.0, a.1, x, y);
                let w2 = edge(a.0, a.1, b.0, b.1, x, y);
                let inside = if area > 0 {
                    w0 >= 0 && w1 >= 0 && w2 >= 0
                } else {
                    w0 <= 0 && w1 <= 0 && w2 <= 0
                };
                if inside {
                    self.px[y as usize * self.w + x as usize] = color;
                }
            }
        }
    }

    pub fn text(&mut self, s: &str, x: i64, y: i64, color: u32, scale: usize) {
        let mut cx = x;
        for ch in s.bytes() {
            let g = glyph(ch);
            for row in 0..7 {
                let bits = g[row];
                for col in 0..5 {
                    if bits & (1 << (4 - col)) != 0 {
                        self.rect(
                            cx + col as i64 * scale as i64,
                            y + row as i64 * scale as i64,
                            scale as i64,
                            scale as i64,
                            color,
                        );
                    }
                }
            }
            cx += 6 * scale as i64;
        }
    }

    pub fn text_width(s: &str, scale: usize) -> i64 {
        s.len() as i64 * 6 * scale as i64
    }
}

fn edge(ax: i64, ay: i64, bx: i64, by: i64, px: i64, py: i64) -> i64 {
    (px - ax) * (by - ay) - (py - ay) * (bx - ax)
}

fn glyph(c: u8) -> [u8; 7] {
    match c {
        b' ' => [0, 0, 0, 0, 0, 0, 0],
        b'A' => [14, 17, 17, 31, 17, 17, 17],
        b'B' => [30, 17, 17, 30, 17, 17, 30],
        b'C' => [14, 17, 16, 16, 16, 17, 14],
        b'D' => [30, 17, 17, 17, 17, 17, 30],
        b'E' => [31, 16, 16, 30, 16, 16, 31],
        b'F' => [31, 16, 16, 30, 16, 16, 16],
        b'G' => [14, 17, 16, 23, 17, 17, 15],
        b'H' => [17, 17, 17, 31, 17, 17, 17],
        b'I' => [14, 4, 4, 4, 4, 4, 14],
        b'J' => [7, 2, 2, 2, 2, 18, 12],
        b'K' => [17, 18, 20, 24, 20, 18, 17],
        b'L' => [16, 16, 16, 16, 16, 16, 31],
        b'M' => [17, 27, 21, 21, 17, 17, 17],
        b'N' => [17, 17, 25, 21, 19, 17, 17],
        b'O' => [14, 17, 17, 17, 17, 17, 14],
        b'P' => [30, 17, 17, 30, 16, 16, 16],
        b'Q' => [14, 17, 17, 17, 21, 18, 13],
        b'R' => [30, 17, 17, 30, 20, 18, 17],
        b'S' => [15, 16, 16, 14, 1, 1, 30],
        b'T' => [31, 4, 4, 4, 4, 4, 4],
        b'U' => [17, 17, 17, 17, 17, 17, 14],
        b'V' => [17, 17, 17, 17, 17, 10, 4],
        b'W' => [17, 17, 17, 21, 21, 27, 17],
        b'X' => [17, 17, 10, 4, 10, 17, 17],
        b'Y' => [17, 17, 10, 4, 4, 4, 4],
        b'Z' => [31, 1, 2, 4, 8, 16, 31],
        b'0' => [14, 17, 19, 21, 25, 17, 14],
        b'1' => [4, 12, 4, 4, 4, 4, 14],
        b'2' => [14, 17, 1, 2, 4, 8, 31],
        b'3' => [31, 2, 4, 6, 1, 17, 14],
        b'4' => [2, 6, 10, 18, 31, 2, 2],
        b'5' => [31, 16, 30, 1, 1, 17, 14],
        b'6' => [6, 8, 16, 30, 17, 17, 14],
        b'7' => [31, 1, 2, 4, 8, 8, 8],
        b'8' => [14, 17, 17, 14, 17, 17, 14],
        b'9' => [14, 17, 17, 15, 1, 2, 12],
        b'_' => [0, 0, 0, 0, 0, 0, 31],
        b'-' => [0, 0, 0, 31, 0, 0, 0],
        b'+' => [0, 4, 4, 31, 4, 4, 0],
        b':' => [0, 4, 4, 0, 4, 4, 0],
        b'.' => [0, 0, 0, 0, 0, 12, 12],
        b'/' => [1, 1, 2, 4, 8, 16, 16],
        b'#' => [10, 31, 10, 10, 31, 10, 0],
        b'(' => [2, 4, 8, 8, 8, 4, 2],
        b')' => [8, 4, 2, 2, 2, 4, 8],
        b'=' => [0, 0, 31, 0, 31, 0, 0],
        b'*' => [0, 10, 4, 31, 4, 10, 0],
        b'%' => [17, 18, 2, 4, 8, 9, 17],
        b'?' => [14, 17, 1, 2, 4, 0, 4],
        _ => [0, 0, 0, 0, 0, 0, 0],
    }
}

pub fn shade(base: u32, k: f64) -> u32 {
    let r = ((base >> 16) & 0xff) as f64 * k;
    let g = ((base >> 8) & 0xff) as f64 * k;
    let b = (base & 0xff) as f64 * k;
    rgb(
        r.clamp(0.0, 255.0) as u8,
        g.clamp(0.0, 255.0) as u8,
        b.clamp(0.0, 255.0) as u8,
    )
}
