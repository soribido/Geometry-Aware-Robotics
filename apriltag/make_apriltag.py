import os
import math
from moms_apriltag import TagGenerator2
from PIL import Image, ImageDraw
import numpy as np

# ====================== Config ======================
TAG_FAMILY = "tag36h11"
TAG_IDS = list(range(0, 12))  #0 12 12 24
TAG_SIZE_MM = 60  # TAG_SIZE(mm)
DPI = 300
PAPER_SIZES_MM = {
    "A4": (210, 297),
    "A3": (297, 420),
}
PAPER_SIZE = "A4"  
MARGIN_MM = 15  # Tag spacing / Paper margin (mm)
GRID_COLS = 3
CUT_MARGIN_RATIO = 0.2  # Ratio of the white margin outside the tag to the thickness of the cutting guide box
# ====================================================


def mm_to_px(mm, dpi=DPI):
    return int(mm / 25.4 * dpi)

def get_tag_and_cell_size_px(tag_size_mm=TAG_SIZE_MM):
    tag_px = mm_to_px(tag_size_mm)
    margin_px = int(tag_px * CUT_MARGIN_RATIO)
    cell_px = tag_px + 2 * margin_px
    return tag_px, margin_px, cell_px

def generate_single_tag_image(tag_id,
                              tag_family=TAG_FAMILY,
                              tag_size_mm=TAG_SIZE_MM):
    tg = TagGenerator2(tag_family)
    tag_np = tg.generate(tag_id) 

    tag_img = Image.fromarray(tag_np).convert("L")

    tag_px, margin_px, cell_px = get_tag_and_cell_size_px(tag_size_mm)
    tag_img = tag_img.resize((tag_px, tag_px), resample=Image.NEAREST)

    canvas = Image.new("L", (cell_px, cell_px), 255)
    offset = margin_px
    canvas.paste(tag_img, (offset, offset))

    draw = ImageDraw.Draw(canvas)

    # Guide box (for cutting)
    cut_offset = margin_px // 2
    x0 = cut_offset
    y0 = cut_offset
    x1 = cell_px - cut_offset - 1
    y1 = cell_px - cut_offset - 1

    draw.rectangle([x0, y0, x1, y1], outline=0, width=1)

    return canvas


def save_individual_tags(tag_ids=TAG_IDS, out_dir="tags_individual"):
    os.makedirs(out_dir, exist_ok=True)
    for tid in tag_ids:
        img = generate_single_tag_image(tid)
        filename = os.path.join(out_dir, f"{TAG_FAMILY}_{tid:04d}.png")
        img.save(filename, dpi=(DPI, DPI))
        print(f"Saved: {filename}")

def make_sheet(tag_ids=TAG_IDS,
               paper_size=PAPER_SIZE,
               filename=None):
    if paper_size not in PAPER_SIZES_MM:
        raise ValueError(f"지원하지 않는 용지 사이즈: {paper_size}")

    if filename is None:
        filename = f"apriltags_{paper_size}_{TAG_FAMILY}_ID_{min(TAG_IDS)}_{max(TAG_IDS)}.png"

    paper_w_mm, paper_h_mm = PAPER_SIZES_MM[paper_size]

    sheet_w = mm_to_px(paper_w_mm)
    sheet_h = mm_to_px(paper_h_mm)

    margin_px = mm_to_px(MARGIN_MM)
    _, _, cell_px = get_tag_and_cell_size_px(TAG_SIZE_MM)

    cols = GRID_COLS
    rows = math.ceil(len(tag_ids) / cols)

    needed_height = margin_px + rows * (cell_px + margin_px)
    if needed_height > sheet_h:
        print("[경고] 태그가 용지 세로 길이를 넘습니다. "
              "TAG_SIZE_MM, MARGIN_MM, GRID_COLS를 줄이거나 A3로 바꿔보세요.")

    sheet = Image.new("L", (sheet_w, sheet_h), 255)

    for idx, tid in enumerate(tag_ids):
        row = idx // cols
        col = idx % cols

        x = margin_px + col * (cell_px + margin_px)
        y = margin_px + row * (cell_px + margin_px)

        if x + cell_px > sheet_w or y + cell_px > sheet_h:
            print(f"[주의] tag id {tid}는 용지 범위를 넘어가서 배치하지 않았습니다.")
            continue

        tag_img = generate_single_tag_image(tid)
        sheet.paste(tag_img, (x, y))

    sheet.save(filename, dpi=(DPI, DPI))
    print(f"{paper_size} 레이아웃 저장 완료: {filename}")


if __name__ == "__main__":
    # 1) 개별 태그 PNG 저장 (가이드 포함)
    save_individual_tags()

    # 2) A4 한 장에 모아서 레이아웃
    make_sheet(paper_size="A4")
