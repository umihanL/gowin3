"""Generate large numeral parking signs for the rescue-maze camera scene.

Only writes parking_1.png, parking_2.png, parking_3.png and the contact sheet
assets/textures/parking_numbers_preview.png. Original geometric artwork.
Run with Python 3 and Pillow; no network services are needed.
"""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent / 'assets' / 'textures'
FONT_PATH = 'C:/Windows/Fonts/arialbd.ttf'
SIZE = 1024
BLUE = '#164d89'
WHITE = '#ffffff'


def centered_text(draw, label, bounds, fill=WHITE, glyph_height=None):
    x0, y0, x1, y1 = bounds
    # Fit actual visible glyph bounds rather than font point size, preserving
    # the numeral proportions and ensuring a true 70%-of-image glyph height.
    height = glyph_height or (y1-y0)
    lo, hi = 1, 1600
    while lo < hi:
        candidate = (lo+hi+1)//2
        f = ImageFont.truetype(FONT_PATH, candidate)
        box = draw.textbbox((0, 0), label, font=f)
        if box[3]-box[1] <= height and box[2]-box[0] <= x1-x0:
            lo = candidate
        else:
            hi = candidate-1
    f = ImageFont.truetype(FONT_PATH, lo)
    box = draw.textbbox((0, 0), label, font=f)
    draw.text(((x0+x1-(box[2]-box[0]))/2-box[0],
               (y0+y1-(box[3]-box[1]))/2-box[1]), label, font=f, fill=fill)
    return box[2]-box[0], box[3]-box[1]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    cards = []
    for numeral in ('1', '2', '3'):
        im = Image.new('RGB', (SIZE, SIZE), BLUE)
        d = ImageDraw.Draw(im)
        d.rounded_rectangle((28, 28, 995, 995), radius=54,
                            outline=WHITE, width=20)
        glyph_size = centered_text(d, numeral, (90, 76, 934, 806),
                                   glyph_height=717)
        centered_text(d, 'PARKING', (135, 878, 889, 945), glyph_height=60)
        path = OUT / f'parking_{numeral}.png'
        im.save(path, optimize=True)
        im.verify()
        cards.append(im)
        print(f'{path.name}: 1024x1024 RGB; numeral {glyph_size[0]}x{glyph_size[1]} pixels')
    preview = Image.new('RGB', (1600, 568), '#e8e8e4')
    for i, im in enumerate(cards):
        im = im.resize((500, 500), Image.Resampling.LANCZOS)
        preview.paste(im, (25+i*525, 16))
        d = ImageDraw.Draw(preview)
        centered_text(d, f'parking_{i+1}.png', (25+i*525, 529, 525+i*525, 553),
                      fill='#283947', glyph_height=24)
    preview.save(OUT / 'parking_numbers_preview.png', optimize=True)


if __name__ == '__main__':
    main()
