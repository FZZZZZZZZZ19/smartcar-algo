#!/usr/bin/env python3
"""生成 docs/手册 里用的机械结构示意图（纯 SVG，无第三方依赖）。
运行：python3 tools/make_diagrams.py
"""
import html
import math
import os

OUT = "docs/images/structures"
os.makedirs(OUT, exist_ok=True)

BODY = "#dbeafe"
BODY_S = "#1d4ed8"
DRIVE = "#111827"
STEER = "#f59e0b"
STEER_S = "#92400e"
CASTER = "#9ca3af"
TEXT = "#111827"
MUTED = "#4b5563"
ARROW = "#dc2626"
DIM = "#2563eb"
BG = "#ffffff"


def esc(s):
    return html.escape(str(s), quote=True)


def text(x, y, s, size=16, fill=TEXT, anchor="start", weight="normal"):
    return (
        f'<text x="{x:.1f}" y="{y:.1f}" font-family="sans-serif" '
        f'font-size="{size}" fill="{fill}" text-anchor="{anchor}" '
        f'font-weight="{weight}">{esc(s)}</text>'
    )


def rect(x, y, w, h, fill, rx=0, stroke=None, sw=2, dash=None, opacity=1):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    st = f' stroke="{stroke}" stroke-width="{sw}"' if stroke else ""
    return (
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
        f'rx="{rx}" fill="{fill}" opacity="{opacity}"{st}{d}/>'
    )


def circle(cx, cy, r, fill, stroke=None, sw=2, dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    st = f' stroke="{stroke}" stroke-width="{sw}"' if stroke else ""
    return f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="{fill}"{st}{d}/>'


def line(x1, y1, x2, y2, stroke, sw=2, dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
        f'stroke="{stroke}" stroke-width="{sw}"{d}/>'
    )


def poly(points, fill, stroke=None, sw=2):
    pts = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    st = f' stroke="{stroke}" stroke-width="{sw}"' if stroke else ""
    return f'<polygon points="{pts}" fill="{fill}"{st}/>'


def arrow(x1, y1, x2, y2, color=ARROW, sw=4, head=14):
    ang = math.atan2(y2 - y1, x2 - x1)
    p2 = (x2 - head * math.cos(ang - 0.42), y2 - head * math.sin(ang - 0.42))
    p3 = (x2 - head * math.cos(ang + 0.42), y2 - head * math.sin(ang + 0.42))
    return (
        line(x1, y1, x2, y2, color, sw)
        + poly([(x2, y2), p2, p3], color)
    )


def wheel(cx, cy, w, h, angle=0, fill=DRIVE, rx=12, stroke=None):
    return (
        f'<g transform="rotate({angle:.1f} {cx:.1f} {cy:.1f})">'
        + rect(cx - w / 2, cy - h / 2, w, h, fill, rx=rx, stroke=stroke)
        + "</g>"
    )


def dim(x1, y1, x2, y2, label, color=DIM, off=0, size=14):
    """尺寸线：两端短竖线 + 中间文字。"""
    mx, my = (x1 + x2) / 2, (y1 + y2) / 2
    dx, dy = x2 - x1, y2 - y1
    ln = math.hypot(dx, dy) or 1
    nx, ny = -dy / ln, dx / ln
    out = [
        line(x1, y1, x2, y2, color, 2),
        line(x1 + nx * 8, y1 + ny * 8, x1 - nx * 8, y1 - ny * 8, color, 2),
        line(x2 + nx * 8, y2 + ny * 8, x2 - nx * 8, y2 - ny * 8, color, 2),
    ]
    out.append(text(mx + nx * (16 + off), my + ny * (16 + off), label,
                    size, color, "middle"))
    return "".join(out)


def svg(name, body, w=800, h=520):
    s = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
        f'viewBox="0 0 {w} {h}">\n'
        f'<rect width="100%" height="100%" fill="{BG}"/>\n'
        + body
        + "\n</svg>\n"
    )
    with open(os.path.join(OUT, name), "w") as f:
        f.write(s)
    print("wrote", os.path.join(OUT, name))


def axis(x, y):
    return (
        arrow(x, y, x, y - 46, MUTED, 3, 10)
        + arrow(x, y, x - 46, y, MUTED, 3, 10)
        + text(x + 6, y - 50, "x（前）", 13, MUTED)
        + text(x - 52, y + 18, "y（左）", 13, MUTED)
    )


# ---------------------------------------------------------------- 差速
def make_diff():
    b = []
    b.append(text(30, 40, "差速驱动（俯视图）", 22, weight="bold"))
    b.append(axis(60, 110))
    b.append(rect(250, 150, 300, 240, BODY, rx=30, stroke=BODY_S))
    b.append(text(400, 260, "车体", 18, MUTED, "middle"))
    # 后驱动轮
    b.append(wheel(225, 340, 36, 92))
    b.append(wheel(575, 340, 36, 92))
    # 前万向轮
    b.append(circle(400, 182, 20, CASTER, "#374151", 3))
    b.append(circle(400, 182, 6, "#374151"))
    b.append(line(400, 150, 400, 164, "#374151", 4))
    # 标签
    b.append(line(150, 340, 206, 340, MUTED, 1, "4 3"))
    b.append(text(140, 332, "驱动轮", 15, TEXT, "end"))
    b.append(text(140, 352, "（电机+编码器）", 12, MUTED, "end"))
    b.append(line(470, 145, 424, 172, MUTED, 1, "4 3"))
    b.append(text(480, 150, "万向轮", 15))
    b.append(text(480, 170, "（只承重，随动）", 12, MUTED))
    # 速度标注
    b.append(text(225, 300, "v_l", 16, DIM, "middle"))
    b.append(text(575, 300, "v_r", 16, DIM, "middle"))
    # 旋转箭头（放在车体下方，避免和万向轮标签打架）
    b.append(
        '<path d="M 384 415 A 16 16 0 1 1 416 415" fill="none" '
        f'stroke="{ARROW}" stroke-width="3"/>'
    )
    b.append(poly([(416, 415), (409, 407), (421, 407)], ARROW))
    b.append(text(430, 421, "w", 16, ARROW))
    # 轮距
    b.append(dim(225, 460, 575, 460, "轮距 L（左右轮中心距）"))
    # 说明
    b.append(text(30, 495, "能前进、后退、原地转；不能横移。转弯靠左右轮速度差。",
                  14, MUTED))
    svg("diff.svg", "".join(b))


# ---------------------------------------------------------------- 阿克曼
def make_ackermann():
    b = []
    b.append(text(30, 40, "阿克曼转向（俯视图）", 22, weight="bold"))
    b.append(axis(60, 110))
    b.append(rect(250, 150, 300, 240, BODY, rx=30, stroke=BODY_S))
    b.append(text(400, 300, "车体", 18, MUTED, "middle"))
    # 后驱动轮
    b.append(wheel(225, 340, 36, 92))
    b.append(wheel(575, 340, 36, 92))
    # 前转向轮：左转，左轮内轮转角更大
    b.append(wheel(225, 220, 34, 88, angle=-32, fill=STEER, stroke=STEER_S))
    b.append(wheel(575, 220, 34, 88, angle=-18, fill=STEER, stroke=STEER_S))
    # 转向连杆
    b.append(line(225, 220, 575, 220, "#92400e", 6))
    b.append(line(225, 220, 250, 250, "#92400e", 4))
    b.append(line(575, 220, 550, 250, "#92400e", 4))
    # 转弯中心与半径
    b.append(circle(70, 340, 7, ARROW))
    b.append(text(52, 366, "O", 18, ARROW, "middle"))
    b.append(line(70, 340, 400, 340, ARROW, 2, "8 6"))
    b.append(text(245, 305, "R", 18, ARROW))
    b.append(line(70, 340, 400, 220, ARROW, 2, "8 6"))
    # 尺寸
    b.append(dim(225, 120, 575, 120, "轮距 W"))
    b.append(dim(660, 220, 660, 340, "轴距 L"))
    # 标签
    b.append(text(140, 205, "内轮转角更大", 14, STEER_S, "end"))
    b.append(line(150, 210, 212, 220, STEER_S, 1, "4 3"))
    b.append(text(660, 400, "前轮：舵机转向", 14, STEER_S))
    b.append(text(660, 422, "后轮：驱动", 14, TEXT))
    b.append(text(30, 495, "不能原地转、不能横移；最小半径 R_min = L / tan(δ_max)。",
                  14, MUTED))
    svg("ackermann.svg", "".join(b))


# ---------------------------------------------------------------- 麦轮
def make_mecanum():
    b = []
    b.append(text(30, 40, "麦克纳姆轮（俯视图）", 22, weight="bold"))
    b.append(axis(60, 110))
    b.append(rect(250, 150, 300, 240, BODY, rx=30, stroke=BODY_S))
    b.append(text(400, 260, "车体", 18, MUTED, "middle"))

    def mech(cx, cy, direction):
        w, h = 38, 96
        out = [rect(cx - w / 2, cy - h / 2, w, h, DRIVE, rx=12)]
        # 5 条 45° 辊子线，左右轮方向镜像
        for k in (-40, -20, 0, 20, 40):
            if direction > 0:
                x1, y1 = cx - 19, cy + k - 19
                x2, y2 = cx + 19, cy + k + 19
            else:
                x1, y1 = cx - 19, cy + k + 19
                x2, y2 = cx + 19, cy + k - 19
            out.append(line(x1, y1, x2, y2, "#d1d5db", 3))
        return "".join(out)

    b.append(mech(225, 200, 1))
    b.append(mech(575, 200, -1))
    b.append(mech(225, 340, 1))
    b.append(mech(575, 340, -1))
    b.append(text(225, 132, "FL", 14, TEXT, "middle"))
    b.append(text(575, 132, "FR", 14, TEXT, "middle"))
    b.append(text(225, 408, "RL", 14, TEXT, "middle"))
    b.append(text(575, 408, "RR", 14, TEXT, "middle"))
    # 运动箭头
    b.append(arrow(400, 130, 400, 80, "#059669"))
    b.append(text(410, 100, "前进 vx", 13, "#059669"))
    b.append(arrow(340, 110, 290, 110, "#059669"))
    b.append(text(300, 98, "横移 vy", 13, "#059669"))
    b.append(
        '<path d="M 350 300 A 55 40 0 1 0 450 300" fill="none" '
        f'stroke="{ARROW}" stroke-width="3"/>'
    )
    b.append(text(400, 360, "原地转 wz", 13, ARROW, "middle"))
    # 轮子细节
    b.append(circle(690, 410, 62, "#f3f4f6", "#374151", 3))
    for k in range(8):
        a = math.radians(k * 45)
        x1 = 690 + 62 * math.cos(a)
        y1 = 410 + 62 * math.sin(a)
        x2 = 690 + 38 * math.cos(a)
        y2 = 410 + 38 * math.sin(a)
        b.append(line(x1, y1, x2, y2, DRIVE, 5))
    b.append(circle(690, 410, 18, "#f3f4f6", "#374151", 3))
    b.append(text(690, 500, "辊子与轮轴成 45°", 12, MUTED, "middle"))
    b.append(text(30, 495, "左右轮辊子镜像安装；四轮必须同时着地。", 14, MUTED))
    svg("mecanum.svg", "".join(b))


# ---------------------------------------------------------------- 全向三轮
def make_omni3():
    b = []
    b.append(text(30, 40, "三轮全向（俯视图，120° 分布）", 22, weight="bold"))
    b.append(axis(60, 110))
    cx, cy = 400, 280
    b.append(circle(cx, cy, 108, BODY, BODY_S, 3))
    b.append(text(cx, cy + 6, "车体", 18, MUTED, "middle"))
    for i, ang_deg in enumerate((90, 210, 330)):
        a = math.radians(ang_deg)
        px = cx + 150 * math.cos(a)
        py = cy - 150 * math.sin(a)
        rot = -(ang_deg + 90)
        b.append(
            f'<g transform="rotate({rot:.1f} {px:.1f} {py:.1f})">'
            + rect(px - 48, py - 15, 96, 30, DRIVE, rx=12)
            + "".join(circle(px - 40 + k * 16, py + 13, 5, "#9ca3af")
                      for k in range(6))
            + "</g>"
        )
        b.append(text(cx + 195 * math.cos(a), cy - 195 * math.sin(a) + 5,
                      f"轮 {i + 1}", 13, MUTED, "middle"))
    b.append(arrow(cx - 120, cy - 120, cx - 120, cy - 175, "#059669"))
    b.append(text(cx - 110, cy - 150, "任意方向", 13, "#059669"))
    b.append(
        f'<path d="M {cx-50} {cy} A 50 50 0 1 0 {cx+50} {cy}" fill="none" '
        f'stroke="{ARROW}" stroke-width="3"/>'
    )
    b.append(text(cx, cy + 70, "原地转", 13, ARROW, "middle"))
    b.append(text(30, 495, "三个轮子间隔 120°，安装角误差会直接让车走偏。",
                  14, MUTED))
    svg("omni3.svg", "".join(b))


# ---------------------------------------------------------------- 履带
def make_tracked():
    b = []
    b.append(text(30, 40, "履带车（侧视图）", 22, weight="bold"))
    # 履带外形
    b.append(rect(240, 270, 340, 100, "none", rx=50, stroke="#374151", sw=26))
    b.append(circle(290, 320, 40, "#4b5563", "#111827", 4))
    b.append(circle(530, 320, 40, "#4b5563", "#111827", 4))
    for x in (370, 420, 470):
        b.append(circle(x, 340, 18, "#6b7280", "#111827", 3))
    b.append(rect(290, 150, 260, 110, BODY, rx=16, stroke=BODY_S))
    b.append(text(420, 212, "车体", 18, MUTED, "middle"))
    b.append(text(290, 430, "导向轮", 14, TEXT, "middle"))
    b.append(text(530, 430, "驱动轮", 14, TEXT, "middle"))
    b.append(text(420, 430, "负重轮", 14, TEXT, "middle"))
    b.append(arrow(420, 130, 420, 80, "#059669"))
    b.append(text(430, 105, "前进", 13, "#059669"))
    b.append(text(30, 495, "转向和差速一样：左右履带速度差；不能横移。",
                  14, MUTED))
    svg("tracked.svg", "".join(b))


# ---------------------------------------------------------------- 舵轮
def make_steerwheel():
    b = []
    b.append(text(30, 40, "舵轮模块（俯视图）", 22, weight="bold"))
    b.append(rect(250, 160, 300, 220, BODY, rx=28, stroke=BODY_S))
    b.append(text(400, 285, "车体", 18, MUTED, "middle"))
    for i, (x, y, ang) in enumerate(
        ((230, 215, 25), (570, 215, -20), (230, 335, -15), (570, 335, 30))
    ):
        b.append(circle(x, y, 26, "#fef3c7", STEER_S, 3))
        b.append(wheel(x, y, 26, 78, angle=ang, fill=STEER,
                       stroke=STEER_S))
        b.append(
            f'<path d="M {x-45} {y-45} A 62 62 0 0 1 {x+45} {y-45}" '
            f'fill="none" stroke="{ARROW}" stroke-width="3"/>'
        )
        ly = y - 72 if i < 2 else y + 72
        b.append(text(x, ly, f"模块 {i+1}", 12, MUTED, "middle"))
    b.append(text(30, 495, "每个模块 = 转向电机 + 驱动电机 + 轮子；四轮独立转向，可全向。",
                  14, MUTED))
    svg("steerwheel.svg", "".join(b))


# ---------------------------------------------------------------- 平衡车
def make_balance():
    b = []
    b.append(text(30, 40, "平衡车（侧视图）", 22, weight="bold"))
    # 轮子
    b.append(circle(400, 400, 46, "#1f2937", "#111827", 4))
    b.append(circle(400, 400, 14, "#9ca3af"))
    # 车身
    b.append(rect(342, 180, 116, 170, BODY, rx=18, stroke=BODY_S))
    # 重心
    b.append(circle(400, 235, 10, ARROW))
    b.append(text(415, 240, "重心", 15, ARROW))
    b.append(arrow(400, 245, 400, 320, ARROW, 3, 12))
    # 倾角
    b.append(line(400, 400, 340, 200, "#6b7280", 2, "7 6"))
    b.append(line(400, 400, 400, 200, "#6b7280", 2, "7 6"))
    b.append(
        f'<path d="M 400 320 A 80 80 0 0 0 372 320" fill="none" '
        f'stroke="{MUTED}" stroke-width="2"/>'
    )
    b.append(text(362, 300, "θ", 16, MUTED))
    b.append(text(400, 470, "同轴两轮（侧视只看到一个）", 14, MUTED, "middle"))
    b.append(text(30, 495, "倒立摆：静态不稳定，必须靠控制保持直立；转向靠左右轮差速。",
                  14, MUTED))
    svg("balance.svg", "".join(b))


# ---------------------------------------------------------------- 六种轮子
def make_wheel_types():
    b = []
    b.append(text(30, 40, "六种常用轮子", 22, weight="bold"))
    cells = [
        ("普通橡胶轮", "只能沿滚动方向"),
        ("万向轮", "360° 随动，不驱动"),
        ("麦克纳姆轮", "45° 辊子，可横移"),
        ("全向轮", "轮缘小滚子，可横移"),
        ("履带", "越野，不能横移"),
        ("舵轮", "转向+驱动一体"),
    ]
    for i, (name, desc) in enumerate(cells):
        col, row = i % 3, i // 3
        x = 40 + col * 250
        y = 80 + row * 220
        b.append(rect(x, y, 220, 190, "#f9fafb", rx=14, stroke="#d1d5db"))
        cx, cy = x + 110, y + 85
        if i == 0:
            b.append(circle(cx, cy, 52, "#1f2937"))
            b.append(circle(cx, cy, 22, "#9ca3af"))
            for k in range(12):
                a = math.radians(k * 30)
                b.append(line(cx + 40 * math.cos(a), cy + 40 * math.sin(a),
                              cx + 52 * math.cos(a), cy + 52 * math.sin(a),
                              "#6b7280", 2))
        elif i == 1:
            b.append(circle(cx, cy - 30, 8, "#4b5563"))
            b.append(line(cx, cy - 30, cx, cy + 10, "#4b5563", 5))
            b.append(circle(cx, cy + 28, 26, "#1f2937"))
            b.append(circle(cx, cy + 28, 8, "#9ca3af"))
            b.append(line(cx - 40, cy - 30, cx + 40, cy - 30, "#9ca3af", 2, "5 5"))
        elif i == 2:
            b.append(circle(cx, cy, 52, "#1f2937"))
            for k in range(8):
                a = math.radians(k * 45)
                rx_, ry_ = 46 * math.cos(a), 46 * math.sin(a)
                dx_ = math.cos(a + math.pi / 4)
                dy_ = math.sin(a + math.pi / 4)
                b.append(line(cx + rx_ - 12 * dx_, cy + ry_ - 12 * dy_,
                              cx + rx_ + 12 * dx_, cy + ry_ + 12 * dy_,
                              "#d1d5db", 6))
            b.append(circle(cx, cy, 18, "#e5e7eb"))
        elif i == 3:
            b.append(circle(cx, cy, 52, "#1f2937"))
            for k in range(12):
                a = math.radians(k * 30)
                b.append(circle(cx + 46 * math.cos(a), cy + 46 * math.sin(a),
                                6, "#9ca3af"))
            b.append(circle(cx, cy, 20, "#e5e7eb"))
        elif i == 4:
            b.append(rect(cx - 55, cy - 32, 110, 64, "none", rx=32,
                          stroke="#374151", sw=20))
            b.append(circle(cx - 38, cy, 20, "#6b7280"))
            b.append(circle(cx + 38, cy, 20, "#6b7280"))
        else:
            b.append(circle(cx, cy + 22, 38, "#1f2937"))
            b.append(circle(cx, cy + 22, 14, "#9ca3af"))
            b.append(rect(cx - 26, cy - 50, 52, 40, STEER, rx=8,
                          stroke=STEER_S))
            b.append(text(cx, cy - 24, "M", 16, "#fff", "middle", "bold"))
        b.append(text(x + 110, y + 150, name, 15, TEXT, "middle", "bold"))
        b.append(text(x + 110, y + 172, desc, 12, MUTED, "middle"))
    svg("wheel-types.svg", "".join(b), 800, 520)


if __name__ == "__main__":
    make_diff()
    make_ackermann()
    make_mecanum()
    make_omni3()
    make_tracked()
    make_steerwheel()
    make_balance()
    make_wheel_types()
