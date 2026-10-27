#!/usr/bin/env python3
"""把仿真 CSV 画成 SVG（只用标准库，不依赖 matplotlib）。

用法：
    python3 tools/plot_sim.py
输出：
    out/sim_compare.svg
    out/sim_compare.html
"""

import csv
import math
import os

OUT = "out"
COLORS = {
    "track": "#4b5563",
    "boundary": "#2f3744",
    "pp": "#56b4e9",
    "stanley": "#e69f00",
    "lqr": "#009e73",
    "text": "#e6e6e6",
    "muted": "#9aa4b2",
    "grid": "#242b36",
    "bg": "#0f1115",
    "panel": "#141922",
}
LABELS = {
    "pp": "纯跟踪 Pure Pursuit",
    "stanley": "Stanley + 曲率前馈",
    "lqr": "LQR + 前馈",
}


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def col(rows, name, step=1):
    return [float(r[name]) for r in rows[::step]]


def nice_ticks(lo, hi, target=5):
    if hi - lo < 1e-9:
        hi = lo + 1.0
    raw = (hi - lo) / target
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        step = m * mag
        if step >= raw:
            break
    start = math.ceil(lo / step) * step
    ticks = []
    v = start
    while v <= hi + 1e-9:
        ticks.append(round(v, 10))
        v += step
    return ticks, step


def fmt(v, step):
    if abs(v) < 1e-9:
        return "0"
    if step >= 1:
        return f"{v:.0f}"
    digits = max(0, -int(math.floor(math.log10(step))))
    return f"{v:.{min(digits + 1, 3)}f}"


class Svg:
    def __init__(self, w, h):
        self.w = w
        self.h = h
        self.parts = []

    def add(self, s):
        self.parts.append(s)

    def rect(self, x, y, w, h, fill, rx=0, stroke=None, sw=1, opacity=1):
        s = (
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" '
            f'rx="{rx}" fill="{fill}" opacity="{opacity}"'
        )
        if stroke:
            s += f' stroke="{stroke}" stroke-width="{sw}"'
        self.add(s + "/>")

    def text(self, x, y, s, size=13, fill=None, anchor="start",
             weight="normal", family="sans-serif"):
        fill = fill or COLORS["text"]
        s = (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))
        self.add(
            f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" '
            f'fill="{fill}" text-anchor="{anchor}" font-weight="{weight}" '
            f'font-family="{family}">{s}</text>'
        )

    def line(self, x1, y1, x2, y2, stroke, sw=1, dash=None, opacity=1):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" '
            f'y2="{y2:.2f}" stroke="{stroke}" stroke-width="{sw}" '
            f'opacity="{opacity}"{d}/>'
        )

    def polyline(self, pts, stroke, sw=2, dash=None, opacity=1):
        if len(pts) < 2:
            return
        d = " ".join(f"{x:.2f},{y:.2f}" for x, y in pts)
        ds = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(
            f'<polyline points="{d}" fill="none" stroke="{stroke}" '
            f'stroke-width="{sw}" stroke-linejoin="round" '
            f'stroke-linecap="round" opacity="{opacity}"{ds}/>'
        )

    def circle(self, x, y, r, fill, stroke=None, sw=1):
        s = f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r}" fill="{fill}"'
        if stroke:
            s += f' stroke="{stroke}" stroke-width="{sw}"'
        self.add(s + "/>")

    def render(self):
        body = "\n".join(self.parts)
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" '
            f'height="{self.h}" viewBox="0 0 {self.w} {self.h}">\n'
            f'<rect width="100%" height="100%" fill="{COLORS["bg"]}"/>\n'
            f"{body}\n</svg>\n"
        )


def map_factory(xmin, xmax, ymin, ymax, px, py, pw, ph):
    sx = pw / (xmax - xmin) if xmax > xmin else 1.0
    sy = ph / (ymax - ymin) if ymax > ymin else 1.0
    s = min(sx, sy)
    ox = px + (pw - (xmax - xmin) * s) / 2.0
    oy = py + (ph - (ymax - ymin) * s) / 2.0

    def m(x, y):
        return (ox + (x - xmin) * s, oy + (ymax - y) * s)

    return m


def draw_track_panel(svg, track, runs, x, y, w, h):
    svg.rect(x, y, w, h, COLORS["panel"], rx=10)

    tx = col(track, "x")
    ty = col(track, "y")

    xmin, xmax = min(tx), max(tx)
    ymin, ymax = min(ty), max(ty)
    pad = 1.2
    m = map_factory(xmin - pad, xmax + pad, ymin - pad, ymax + pad,
                    x + 20, y + 30, w - 40, h - 55)

    # 赛道边界（示意路面宽度）
    left, right = [], []
    n = len(tx)
    for i in range(n):
        x0, y0 = tx[i], ty[i]
        x1, y1 = tx[(i + 1) % n], ty[(i + 1) % n]
        dx, dy = x1 - x0, y1 - y0
        ln = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / ln, dx / ln
        half = 0.9
        left.append(m(x0 + nx * half, y0 + ny * half))
        right.append(m(x0 - nx * half, y0 - ny * half))

    svg.polyline(left + [left[0]], COLORS["boundary"], 1.2, opacity=0.9)
    svg.polyline(right + [right[0]], COLORS["boundary"], 1.2, opacity=0.9)

    # 中线
    center = [m(px_, py_) for px_, py_ in zip(tx, ty)]
    svg.polyline(center + [center[0]], COLORS["track"], 1.6, dash="6 6",
                 opacity=0.9)

    # 三条轨迹
    for key, rows in runs.items():
        if not rows:
            continue
        pts = [m(float(r["x"]), float(r["y"])) for r in rows[::6]]
        svg.polyline(pts, COLORS[key], 2.0, opacity=0.95)

    # 起点
    sx, sy = m(tx[0], ty[0])
    svg.circle(sx, sy, 5, "#ffffff", COLORS["bg"], 2)

    svg.text(x + 16, y + 22, "赛道与轨迹（俯视）", 15, weight="bold")
    svg.text(x + w - 16, y + 22, "起点 ●", 11, COLORS["muted"], "end")

    # 图例（左下角）
    lx = x + 18
    ly = y + h - 18
    short = {"pp": "纯跟踪", "stanley": "Stanley+FF", "lqr": "LQR+FF"}
    for key in ("pp", "stanley", "lqr"):
        if key not in runs:
            continue
        svg.line(lx, ly - 4, lx + 20, ly - 4, COLORS[key], 3)
        svg.text(lx + 26, ly, short[key], 11, COLORS["text"])
        lx += 125

    draw_zoom_inset(svg, track, runs, x + w - 190, y + h - 168, 176, 148)


def draw_zoom_inset(svg, track, runs, x, y, w, h):
    """在最急弯处按弧长窗口放大，展示三条轨迹的细微差别。"""
    kappa = col(track, "kappa")
    idx = max(range(len(kappa)), key=lambda i: abs(kappa[i]))

    s_vals = [float(r["s"]) for r in track]
    ds = s_vals[1] - s_vals[0] if len(s_vals) > 1 else 0.1
    total = s_vals[-1] + ds
    s0 = s_vals[idx]
    half = 1.3

    def near(sv):
        d = abs(sv - s0) % total
        return min(d, total - d) <= half

    seg = [r for r in track if near(float(r["s"]))]
    if len(seg) < 3:
        return

    xs = [float(r["x"]) for r in seg]
    ys = [float(r["y"]) for r in seg]
    pad = 0.35

    svg.rect(x, y, w, h, "#0b0e13", rx=8, stroke="#2b3442", sw=1)
    m = map_factory(min(xs) - pad, max(xs) + pad, min(ys) - pad,
                    max(ys) + pad, x + 6, y + 22, w - 12, h - 28)

    svg.polyline([m(float(r["x"]), float(r["y"])) for r in seg],
                 COLORS["track"], 1.4, dash="4 4", opacity=0.9)

    # 让误差最大的纯跟踪画在最上层
    for key in ("lqr", "stanley", "pp"):
        rows = runs.get(key, [])
        sub = []
        for r in rows[::2]:
            if near(float(r["s"])):
                sub.append(m(float(r["x"]), float(r["y"])))
        if len(sub) > 1:
            svg.polyline(sub, COLORS[key], 1.8)

    svg.text(x + 8, y + 15, "最急弯放大", 10, COLORS["muted"])


def draw_series(svg, x, y, w, h, title, series, ylabel, tmax):
    svg.rect(x, y, w, h, COLORS["panel"], rx=10)
    svg.text(x + 14, y + 22, title, 14, weight="bold")

    pad_l, pad_r, pad_t, pad_b = 52, 14, 34, 30
    px, py, pw, ph = x + pad_l, y + pad_t, w - pad_l - pad_r, h - pad_t - pad_b

    lo = min((min(v) for v in series.values() if v), default=0.0)
    hi = max((max(v) for v in series.values() if v), default=1.0)
    if hi - lo < 0.02:
        c = 0.5 * (hi + lo)
        lo, hi = c - 0.01, c + 0.01
    span = hi - lo
    lo -= 0.08 * span
    hi += 0.08 * span

    ticks, step = nice_ticks(lo, hi, 4)
    for tv in ticks:
        yy = py + ph - (tv - lo) / (hi - lo) * ph
        svg.line(px, yy, px + pw, yy, COLORS["grid"], 1)
        svg.text(px - 8, yy + 4, fmt(tv, step), 10, COLORS["muted"], "end")

    if lo < 0 < hi:
        yz = py + ph - (0 - lo) / (hi - lo) * ph
        svg.line(px, yz, px + pw, yz, "#3a4454", 1, dash="3 3")

    xt, xstep = nice_ticks(0, tmax, 4)
    for tv in xt:
        xx = px + (tv / tmax) * pw
        svg.line(xx, py, xx, py + ph, COLORS["grid"], 1, opacity=0.6)
        svg.text(xx, py + ph + 16, fmt(tv, xstep), 10, COLORS["muted"],
                 "middle")

    svg.text(px + pw / 2, y + h - 6, "时间 (s)", 10, COLORS["muted"],
             "middle")

    for key, vals in series.items():
        if not vals:
            continue
        pts = []
        tstep = tmax / max(1, len(vals) - 1)
        for i, v in enumerate(vals):
            xx = px + (i * tstep / tmax) * pw
            yy = py + ph - (v - lo) / (hi - lo) * ph
            pts.append((xx, yy))
        svg.polyline(pts, COLORS[key], 1.8)


def main():
    os.makedirs(OUT, exist_ok=True)

    track = read_csv(os.path.join(OUT, "track.csv"))
    runs = {
        "pp": read_csv(os.path.join(OUT, "pp.csv")),
        "stanley": read_csv(os.path.join(OUT, "stanley.csv")),
        "lqr": read_csv(os.path.join(OUT, "lqr.csv")),
    }

    if not track:
        print("缺少 out/track.csv，先运行 make sim")
        return

    tmax = 24.0
    series_ey = {k: col(v, "ey", 4) for k, v in runs.items()}
    series_v = {k: col(v, "v", 4) for k, v in runs.items()}

    svg = Svg(1000, 740)
    svg.text(24, 32, "智能车横向控制算法对比", 22, weight="bold")
    svg.text(24, 52, "同一车辆模型 / 同一赛道 / 同一速度剖面，控制周期 5 ms",
             12, COLORS["muted"])

    draw_track_panel(svg, track, runs, 20, 70, 600, 640)
    draw_series(svg, 640, 70, 340, 300, "横向误差 e_y (m)", series_ey, "m", tmax)
    draw_series(svg, 640, 390, 340, 320, "纵向速度 v (m/s)", series_v, "m/s", tmax)

    svg_path = os.path.join(OUT, "sim_compare.svg")
    with open(svg_path, "w") as f:
        f.write(svg.render())

    html = f"""<!doctype html>
<html lang="zh-CN">
<meta charset="utf-8">
<title>智能车横向控制算法对比</title>
<style>
  body {{ margin: 0; background: {COLORS['bg']}; }}
  .wrap {{ max-width: 1000px; margin: 0 auto; }}
</style>
<div class="wrap">
{svg.render()}
</div>
</html>
"""
    with open(os.path.join(OUT, "sim_compare.html"), "w") as f:
        f.write(html)

    print("已生成 out/sim_compare.svg 和 out/sim_compare.html")


if __name__ == "__main__":
    main()
