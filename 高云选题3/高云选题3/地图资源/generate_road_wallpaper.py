"""Generate a clean, top-down road wallpaper for the rescue maze.

The output is a visual ground layer only.  It does not alter collision geometry,
thin walls, navigation, or sensor models.  Coordinates are metres with north at
positive Y; the PNG places north at the top and world X horizontally.
"""

from __future__ import annotations

import json
import math
import random
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

from PIL import Image, ImageChops, ImageDraw, ImageFilter, ImageFont, ImageCms


ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "rescue_maze_config.json"
TEXTURE_DIR = ROOT / "assets" / "textures"
ASSET_DIR = ROOT / "assets"
OUT_PATH = ASSET_DIR / "road_wallpaper.png"
PREVIEW_PATH = ASSET_DIR / "road_wallpaper_preview.png"

SITE_W_M = 12.0
SITE_H_M = 8.0
OUT_W = 4608
OUT_H = 3072
PREVIEW_W = 1152
PREVIEW_H = 768
PX_PER_M = OUT_W / SITE_W_M

ROAD_DARK = (52, 58, 61)
ROAD_DARK_ALT = (57, 63, 66)
COURTYARD = (166, 171, 172)
COURTYARD_LINE = (184, 188, 188)
PARKING = (87, 113, 128)
PARKING_ALT = (92, 119, 133)
WHITE_LINE = (235, 238, 232)
YELLOW_LINE = (223, 207, 150)
ZEBRA = (245, 246, 238)
ARROW = (242, 244, 235)
DETOUR = (243, 190, 73)
WALL_FOOT = (112, 119, 121)


def load_font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    candidates = [
        Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
        Path("C:/Windows/Fonts/segoeuib.ttf" if bold else "C:/Windows/Fonts/segoeui.ttf"),
        Path("C:/Windows/Fonts/NotoSansSC-VF.ttf"),
    ]
    for path in candidates:
        if path.exists():
            try:
                return ImageFont.truetype(str(path), size)
            except OSError:
                pass
    return ImageFont.load_default()


def xy_to_px(x: float, y: float) -> Tuple[int, int]:
    return int(round(x * PX_PER_M)), int(round(OUT_H - y * PX_PER_M))


def rect_px(x0: float, y0: float, x1: float, y1: float) -> Tuple[int, int, int, int]:
    # World bounds use y0 < y1; image y is inverted.
    ax, ay = xy_to_px(x0, y0)
    bx, by = xy_to_px(x1, y1)
    return min(ax, bx), min(ay, by), max(ax, bx), max(ay, by)


def polygon_px(points: Sequence[Sequence[float]]) -> List[Tuple[int, int]]:
    return [xy_to_px(float(x), float(y)) for x, y in points]


def draw_world_line(draw: ImageDraw.ImageDraw, a: Sequence[float], b: Sequence[float], fill, width_m: float) -> None:
    draw.line([xy_to_px(*a), xy_to_px(*b)], fill=fill, width=max(1, int(round(width_m * PX_PER_M))), joint="curve")


def draw_world_rect(draw: ImageDraw.ImageDraw, bounds: Sequence[float], fill, outline=None, width_m: float = 0.0) -> None:
    draw.rectangle(rect_px(*bounds), fill=fill, outline=outline, width=max(1, int(round(width_m * PX_PER_M))) if outline else 1)


def centred_text(draw: ImageDraw.ImageDraw, x: float, y: float, text: str, font, fill, stroke=0, stroke_fill=(0, 0, 0)) -> None:
    px, py = xy_to_px(x, y)
    box = draw.textbbox((0, 0), text, font=font, stroke_width=stroke)
    draw.text((px - (box[2] - box[0]) / 2, py - (box[3] - box[1]) / 2 - box[1]), text, font=font, fill=fill, stroke_width=stroke, stroke_fill=stroke_fill)


def make_asphalt(size: Tuple[int, int]) -> Image.Image:
    """Make subtle, low-contrast, tile-free asphalt grain."""
    rng = random.Random(20261003)
    base = Image.new("RGB", size, ROAD_DARK)
    fine = Image.effect_noise(size, 20).convert("RGB")
    fine = ImageChops.multiply(fine, Image.new("RGB", size, (190, 190, 190)))
    base = Image.blend(base, fine, 0.095)
    # A blurred field keeps large areas from looking like a flat digital block.
    sw, sh = max(8, size[0] // 48), max(8, size[1] // 48)
    coarse = Image.effect_noise((sw, sh), 18).resize(size, Image.Resampling.BICUBIC).convert("RGB")
    base = Image.blend(base, coarse, 0.045)
    speckles = Image.new("RGBA", size, (0, 0, 0, 0))
    sd = ImageDraw.Draw(speckles)
    count = max(200, size[0] * size[1] // 170000)
    for _ in range(count):
        x = rng.randrange(size[0])
        y = rng.randrange(size[1])
        r = rng.choice((1, 1, 2, 3))
        col = (125, 132, 132, rng.randrange(8, 22))
        sd.ellipse((x - r, y - r, x + r, y + r), fill=col)
    return Image.alpha_composite(base.convert("RGBA"), speckles).convert("RGB")


def dashed_segments(start: float, end: float, dash: float, gap: float, skip: Iterable[Tuple[float, float]]) -> Iterable[Tuple[float, float]]:
    """Yield portions of a 1-D line that survive junction/crosswalk masks."""
    skips = sorted((min(a, b), max(a, b)) for a, b in skip if b > a)
    pos = start
    on = True
    while pos < end - 1e-9:
        if not on and gap <= 0:
            on = True
            continue
        nxt = min(end, pos + (dash if on else gap))
        if on:
            pieces = [(pos, nxt)]
            for a, b in skips:
                pieces = [(u, v) for p, q in pieces for u, v in ((p, min(q, a)), (max(p, b), q)) if v > u]
            yield from pieces
        pos = nxt
        on = not on


def draw_h_dashed(draw: ImageDraw.ImageDraw, y: float, skips: Iterable[Tuple[float, float]], width_m: float = 0.025) -> None:
    for x0, x1 in dashed_segments(0.15, SITE_W_M - 0.15, 0.34, 0.28, skips):
        draw_world_line(draw, (x0, y), (x1, y), YELLOW_LINE, width_m)


def draw_v_dashed(draw: ImageDraw.ImageDraw, x: float, skips: Iterable[Tuple[float, float]], width_m: float = 0.025) -> None:
    for y0, y1 in dashed_segments(0.15, SITE_H_M - 0.15, 0.34, 0.28, skips):
        draw_world_line(draw, (x, y0), (x, y1), YELLOW_LINE, width_m)


def draw_arrow(draw: ImageDraw.ImageDraw, x: float, y: float, heading: str, length: float = 0.52, color=ARROW) -> None:
    """A broad top-down direction arrow, with heading +X/-X/+Y/-Y."""
    half = 0.08
    head = 0.18
    if heading == "+x":
        pts = [(x - length / 2, y - half / 2), (x + length / 2 - head, y - half / 2), (x + length / 2 - head, y - half), (x + length / 2, y), (x + length / 2 - head, y + half), (x + length / 2 - head, y + half / 2), (x - length / 2, y + half / 2)]
    elif heading == "-x":
        pts = [(x + length / 2, y - half / 2), (x - length / 2 + head, y - half / 2), (x - length / 2 + head, y - half), (x - length / 2, y), (x - length / 2 + head, y + half), (x - length / 2 + head, y + half / 2), (x + length / 2, y + half / 2)]
    elif heading == "+y":
        pts = [(x - half / 2, y - length / 2), (x - half / 2, y + length / 2 - head), (x - half, y + length / 2 - head), (x, y + length / 2), (x + half, y + length / 2 - head), (x + half / 2, y + length / 2 - head), (x + half / 2, y - length / 2)]
    else:
        pts = [(x - half / 2, y + length / 2), (x - half / 2, y - length / 2 + head), (x - half, y - length / 2 + head), (x, y - length / 2), (x + half, y - length / 2 + head), (x + half / 2, y - length / 2 + head), (x + half / 2, y + length / 2)]
    draw.polygon(polygon_px(pts), fill=color)


def draw_crosswalk(draw: ImageDraw.ImageDraw, item: dict) -> None:
    cx, cy = item["center"]
    axis = item["axis"].upper()
    road_width = float(item["road_width"])
    length = float(item["length"])
    stripe_w = 0.11
    count = max(4, int(round(length / 0.14)))
    stripe_len = road_width * 0.92
    for i in range(count):
        t = (i + 0.5) / count
        offset = (t - 0.5) * length
        if axis == "Y":
            draw_world_rect(draw, (cx - stripe_len / 2, cy + offset - stripe_w / 2, cx + stripe_len / 2, cy + offset + stripe_w / 2), ZEBRA)
        else:
            draw_world_rect(draw, (cx + offset - stripe_w / 2, cy - stripe_len / 2, cx + offset + stripe_w / 2, cy + stripe_len / 2), ZEBRA)


def draw_courtyards(draw: ImageDraw.ImageDraw, cfg: dict) -> None:
    for courtyard in cfg.get("courtyards", []):
        xmin, xmax, ymin, ymax = courtyard["bounds"]
        bounds = (xmin, ymin, xmax, ymax)
        draw_world_rect(draw, bounds, COURTYARD)
        # Sparse paving grid, clipped by the courtyard rectangle.
        x0, y0, x1, y1 = bounds
        step = 0.32
        x = x0 + step
        while x < x1 - 1e-6:
            draw_world_line(draw, (x, y0), (x, y1), COURTYARD_LINE, 0.008)
            x += step
        y = y0 + step
        while y < y1 - 1e-6:
            draw_world_line(draw, (x0, y), (x1, y), COURTYARD_LINE, 0.008)
            y += step


def draw_parking(draw: ImageDraw.ImageDraw, cfg: dict) -> None:
    parking = cfg["parking"]
    xmin, xmax, ymin, ymax = parking["bounds"]
    bounds = (xmin, ymin, xmax, ymax)
    draw_world_rect(draw, bounds, PARKING)
    # Very restrained blue-grey bands; no decorative pattern at wall feet.
    x0, y0, x1, y1 = bounds
    band = 0.24
    y = y0 + band
    while y < y1:
        draw_world_line(draw, (x0, y), (x1, y), PARKING_ALT, 0.012)
        y += band
    bw, bh = parking["bay_size"]
    pfont = load_font(136, True)
    nfont = load_font(66, True)
    for idx, (cx, cy) in enumerate(parking["bay_centers"], start=1):
        b = (cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2)
        draw_world_rect(draw, b, None, outline=WHITE_LINE, width_m=0.035)
        centred_text(draw, cx - 0.27, cy, "P", pfont, WHITE_LINE, stroke=2, stroke_fill=(47, 66, 76))
        centred_text(draw, cx + 0.35, cy, str(idx), nfont, WHITE_LINE, stroke=1, stroke_fill=(47, 66, 76))
    # Parking entrance: clear opening at east edge, with short white jamb marks.
    draw_world_line(draw, (x1, 4.79), (x1, 5.15), WHITE_LINE, 0.035)
    draw_world_line(draw, (x1, 6.65), (x1, 7.86), WHITE_LINE, 0.035)
    draw_world_line(draw, (x1 - 0.18, 5.15), (x1 - 0.02, 5.15), WHITE_LINE, 0.035)
    draw_world_line(draw, (x1 - 0.18, 6.65), (x1 - 0.02, 6.65), WHITE_LINE, 0.035)


def draw_road_markings(draw: ImageDraw.ImageDraw, cfg: dict) -> None:
    # Main street widths leave safe clearance for the assumed 0.48 x 0.42 m robot.
    h_roads = [(1.05, 1.00), (4.00, 1.80), (7.25, 1.00)]
    v_roads = [(1.05, 1.00), (6.00, 1.80), (10.85, 1.00)]
    h_crosswalk_skips = []
    v_crosswalk_skips = []
    for crosswalk in cfg.get("crosswalks", []):
        cx, cy = crosswalk["center"]
        half = float(crosswalk["length"]) / 2 + 0.12
        if crosswalk["axis"].upper() == "X":
            h_crosswalk_skips.append((cx - half, cx + half))
        else:
            v_crosswalk_skips.append((cy - half, cy + half))
    for y, width in h_roads:
        edge = max(0.06, width / 2 - 0.10)
        # Interrupt edge lines at perpendicular roads and the top parking surface.
        skip = [(x - w / 2 - 0.10, x + w / 2 + 0.10) for x, w in v_roads]
        if abs(y - 7.25) < 1e-6:
            skip.append((2.0, 4.82))
        skip.extend(h_crosswalk_skips)
        draw_h_dashed(draw, y, skip)
        for sy in (y - edge, y + edge):
            for x0, x1 in dashed_segments(0.15, SITE_W_M - 0.15, 99, 0, skip):
                draw_world_line(draw, (x0, sy), (x1, sy), WHITE_LINE, 0.026)
    for x, width in v_roads:
        edge = max(0.06, width / 2 - 0.10)
        skip = [(y - w / 2 - 0.10, y + w / 2 + 0.10) for y, w in h_roads]
        skip.extend(v_crosswalk_skips)
        center_skips = skip + ([(5.1, 5.95)] if abs(x-6.0) < 1e-6 else [])
        draw_v_dashed(draw, x, center_skips)
        for sx in (x - edge, x + edge):
            for y0, y1 in dashed_segments(0.15, SITE_H_M - 0.15, 99, 0, skip):
                draw_world_line(draw, (sx, y0), (sx, y1), WHITE_LINE, 0.026)

    for crosswalk in cfg.get("crosswalks", []):
        draw_crosswalk(draw, crosswalk)

    # Two large direction arrows for a readable small-car view.
    draw_arrow(draw, 1.27, 2.10, "+y", length=0.56)
    draw_arrow(draw, 8.95, 3.65, "+x", length=0.62)

    # Construction detour: right-pointing arrow at the requested world coordinate.
    draw_arrow(draw, 6.0, 5.60, "+x", length=0.62, color=DETOUR)
    centred_text(draw, 6.0, 5.30, "DETOUR", load_font(60, True), DETOUR, stroke=2, stroke_fill=(48, 56, 58))


def draw_wall_feet(draw: ImageDraw.ImageDraw, cfg: dict) -> None:
    # Neutral thin strips at wall bases keep patterned ground from running under walls.
    for wall in cfg.get("walls", []):
        draw_world_line(draw, wall["a"], wall["b"], WALL_FOOT, 0.075)


def add_metadata(img: Image.Image) -> Image.Image:
    try:
        profile = ImageCms.ImageCmsProfile(ImageCms.createProfile("sRGB")).tobytes()
    except Exception:
        profile = None
    if profile:
        img.info["icc_profile"] = profile
    img.info["Description"] = "Parametric rescue maze road wallpaper; 12x8 m assumption, north-up, X horizontal; visual simulation only."
    img.info["Software"] = "generate_road_wallpaper.py / Pillow"
    return img


def generate() -> Tuple[Path, Path]:
    cfg = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    site = cfg.get("site", [SITE_W_M, SITE_H_M])
    if tuple(site) != (SITE_W_M, SITE_H_M):
        raise ValueError(f"Expected 12x8 m output mapping, got site={site}")

    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    img = make_asphalt((OUT_W, OUT_H))
    draw = ImageDraw.Draw(img)
    # Draw broad asphalt streets first; parking and courtyard surfaces override them.
    for y, width in ((1.05, 1.00), (4.00, 1.80), (7.25, 1.00)):
        draw_world_rect(draw, (0.0, y - width / 2, SITE_W_M, y + width / 2), ROAD_DARK_ALT)
    for x, width in ((1.05, 1.00), (6.00, 1.80), (10.85, 1.00)):
        draw_world_rect(draw, (x - width / 2, 0.0, x + width / 2, SITE_H_M), ROAD_DARK_ALT)

    draw_courtyards(draw, cfg)
    draw_parking(draw, cfg)
    draw_road_markings(draw, cfg)
    draw_wall_feet(draw, cfg)

    # Small, quiet orientation ticks make UV inspection easier without adding clutter.
    draw_world_line(draw, (0.2, 0.2), (0.55, 0.2), (214, 219, 216), 0.012)
    draw_world_line(draw, (0.2, 0.2), (0.2, 0.55), (214, 219, 216), 0.012)

    img = add_metadata(img)
    img.save(OUT_PATH, format="PNG", optimize=True, compress_level=6)
    preview = img.resize((PREVIEW_W, PREVIEW_H), Image.Resampling.LANCZOS)
    preview = add_metadata(preview)
    preview.save(PREVIEW_PATH, format="PNG", optimize=True, compress_level=6)
    return OUT_PATH, PREVIEW_PATH


if __name__ == "__main__":
    output, preview = generate()
    print(json.dumps({"wallpaper": str(output), "preview": str(preview), "size": [OUT_W, OUT_H], "mode": "RGB", "units": "m", "site": [SITE_W_M, SITE_H_M]}, ensure_ascii=False))
