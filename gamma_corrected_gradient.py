"""
gamma_corrected_gradient.py
---------------------------
Generates a gamma-corrected grayscale shade list (and optional PDF
gradients) designed so the 3Pi+ line sensor produces a *linear*
raw-ADC response across the shade index.

Given the fitted per-sensor power-law  raw = A - B·(L/100)^n
(see transfer_function.py / TransferFunction_Analysis.md), evenly
spaced raw-ADC targets require the printed lightness to follow

        L_i = 100 · (i / (N-1)) ^ (1 / n)

i.e. the printed gradient is pre-distorted by the inverse of the
sensor power law.  The exponent defaults to the mean of the per-sensor
CAL_N values parsed out of the active Calibration.h; override with
--n if desired.

Outputs (in --out-dir, default `data/`):

  <stem>_gamma_shades.csv      — per-shade table (index, L% for uniform
                                 and gamma-corrected plans, 8-bit sRGB)
  <stem>_gamma_gradient.pdf    — printable strip of N gamma-corrected
                                 grey patches
  <stem>_uniform_vs_gamma.pdf  — two strips stacked for visual A/B

Usage:
  python gamma_corrected_gradient.py                  # defaults (N=26, n from Calibration.h)
  python gamma_corrected_gradient.py --shades 12 --n 2.84
  python gamma_corrected_gradient.py --no-pdf         # CSV only
"""

import argparse
import csv
import os
import re
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_pdf import PdfPages


DEFAULT_CAL_PATH = os.path.join("GradientCentreFinder", "Calibration.h")
DEFAULT_OUT_DIR  = "data"
DEFAULT_SHADES   = 26

# Physical units — the printable gradient is rendered at true scale
# so each printed shade block is exactly 110 mm × 40 mm.
MM_PER_INCH = 25.4
A3_W_MM     = 297.0        # A3 portrait width
A3_H_MM     = 420.0        # A3 portrait height
A3_PORTRAIT_IN  = (A3_W_MM / MM_PER_INCH, A3_H_MM / MM_PER_INCH)   # ≈ 11.69 × 16.54
A3_LANDSCAPE_IN = (A3_H_MM / MM_PER_INCH, A3_W_MM / MM_PER_INCH)   # ≈ 16.54 × 11.69

BOX_W_MM = 110.0           # shade block — long edge (drive direction)
BOX_H_MM = 40.0            # shade block — short edge

# Page margins and inter-box spacing for the multi-page gradient.
MARGIN_TOP_MM    = 20.0
MARGIN_BOTTOM_MM = 14.0
MARGIN_SIDE_MM   = 12.0
LABEL_COL_MM     = 18.0    # label strip to the left of every block
ROW_GAP_MM       = 5.0     # vertical space between blocks (kept clear)


# -----------------------------------------------------------------------
# Parse CAL_N from the active Calibration.h (if available)
# -----------------------------------------------------------------------
def read_exponent_from_header(path):
    """Return (mean_n, per_sensor_n) parsed from a Calibration.h,
    or (None, None) if the file / CAL_N array is missing."""
    if not os.path.isfile(path):
        return None, None
    text = open(path, "r", encoding="utf-8").read()
    m = re.search(r"CAL_N\s*\[\s*\d+\s*\]\s*=\s*\{\s*([^}]*)\}", text)
    if not m:
        return None, None
    try:
        vals = [float(v.strip()) for v in m.group(1).split(",") if v.strip()]
    except ValueError:
        return None, None
    if not vals:
        return None, None
    return float(np.mean(vals)), vals


# -----------------------------------------------------------------------
# Compute shade table
# -----------------------------------------------------------------------
def compute_shade_table(n_shades, n_exp):
    """Return a list of dicts, one per shade."""
    rows = []
    for i in range(n_shades):
        f = i / (n_shades - 1) if n_shades > 1 else 0.0
        L_uniform = 100.0 * f                          # old plan
        L_gamma   = 100.0 * f ** (1.0 / n_exp)          # new plan
        # 8-bit sRGB value assumed: gradient fill treated as
        # percentage-of-white in the same space in which the original
        # PDF was authored.
        srgb_gamma = int(round(255 * L_gamma / 100.0))
        rows.append({
            "shade":        i,
            "L_uniform_%":  L_uniform,
            "L_gamma_%":    L_gamma,
            "sRGB_8bit":    srgb_gamma,
            "sRGB_hex":     f"#{srgb_gamma:02X}{srgb_gamma:02X}{srgb_gamma:02X}",
        })
    return rows


# -----------------------------------------------------------------------
# CSV output
# -----------------------------------------------------------------------
def write_csv(rows, out_path, n_exp, n_shades):
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        f.write(f"# gamma-corrected shade table — N={n_shades}, n={n_exp:.4f}\n")
        f.write("# L_uniform_%: original evenly-spaced plan (0..100)\n")
        f.write("# L_gamma_%  : print this lightness to get a linear ADC step\n")
        f.write("# sRGB_8bit  : assume PDF fill = L_gamma / 100 in same space as original gradient\n")
        writer = csv.DictWriter(
            f,
            fieldnames=["shade", "L_uniform_%", "L_gamma_%", "sRGB_8bit", "sRGB_hex"],
        )
        writer.writeheader()
        for r in rows:
            writer.writerow({
                "shade":        r["shade"],
                "L_uniform_%":  f"{r['L_uniform_%']:.2f}",
                "L_gamma_%":    f"{r['L_gamma_%']:.2f}",
                "sRGB_8bit":    r["sRGB_8bit"],
                "sRGB_hex":     r["sRGB_hex"],
            })
    print(f"Wrote {out_path}")


# -----------------------------------------------------------------------
# Single-strip PDF (the printable gradient)
# -----------------------------------------------------------------------
def _compute_layout():
    """Return (cols, rows_per_page, boxes_per_page, col_pitch_mm,
    row_pitch_mm) for the print-true gradient layout on A3 portrait.

    Each cell = [LABEL_COL_MM | BOX_W_MM].  A block is exactly
    110 mm × 40 mm at print time; nothing is drawn on top of it.
    """
    cell_w     = LABEL_COL_MM + BOX_W_MM
    usable_w   = A3_W_MM - 2 * MARGIN_SIDE_MM
    cols       = max(1, int(usable_w // cell_w))
    col_pitch  = cell_w

    usable_h   = A3_H_MM - MARGIN_TOP_MM - MARGIN_BOTTOM_MM
    row_pitch  = BOX_H_MM + ROW_GAP_MM
    rows_pp    = max(1, int((usable_h + ROW_GAP_MM) // row_pitch))
    return cols, rows_pp, cols * rows_pp, col_pitch, row_pitch


def render_gradient_pdf(rows, out_path, title, key="L_gamma_%"):
    """Multi-page A3-portrait PDF with physically-sized shade blocks
    (110 mm × 40 mm).  All index / lightness labels sit in the left
    margin column — no ink is ever placed on top of a shade block,
    so the IR line sensors see a clean patch of grey.

    `key` selects which column of `rows` to render (L_gamma_% by
    default; can be set to L_uniform_% to render the old plan).
    """
    n = len(rows)
    cols, rows_pp, per_page, col_pitch, row_pitch = _compute_layout()
    num_pages = (n + per_page - 1) // per_page

    with PdfPages(out_path) as pdf:
        for page_idx in range(num_pages):
            fig = plt.figure(figsize=A3_PORTRAIT_IN)
            # Single axes covering the whole sheet, with data units in
            # millimetres.  y inverted so the first block is at top-left.
            ax = fig.add_axes([0, 0, 1, 1])
            ax.set_xlim(0, A3_W_MM)
            ax.set_ylim(A3_H_MM, 0)
            ax.set_aspect("equal")
            ax.axis("off")

            # Page title (stays in the top margin — well clear of blocks)
            ax.text(A3_W_MM / 2, MARGIN_TOP_MM / 2,
                    f"{title}   —   page {page_idx + 1} / {num_pages}",
                    ha="center", va="center", fontsize=10, color="black")

            page_start = page_idx * per_page
            for local_i in range(per_page):
                global_i = page_start + local_i
                if global_i >= n:
                    break
                row = local_i // cols
                col = local_i % cols

                x = (MARGIN_SIDE_MM
                     + col * col_pitch
                     + LABEL_COL_MM)
                y = MARGIN_TOP_MM + row * row_pitch

                g = rows[global_i][key] / 100.0
                ax.add_patch(Rectangle(
                    (x, y), BOX_W_MM, BOX_H_MM,
                    facecolor=(g, g, g), edgecolor="none",
                ))

                # Label strip (LEFT of the block).  Vertically centred
                # on the block so it's obvious which label belongs to
                # which patch.  Never overlaps the block itself.
                ax.text(x - 2.0, y + BOX_H_MM / 2,
                        f"#{global_i}\n{rows[global_i][key]:.1f} %",
                        ha="right", va="center", fontsize=9,
                        color="black")

            # Footer with print-integrity reminder (outside all blocks).
            ax.text(A3_W_MM / 2, A3_H_MM - MARGIN_BOTTOM_MM / 2,
                    f"Each block = {BOX_W_MM:.0f} mm × {BOX_H_MM:.0f} mm. "
                    f"Print at 100 % scale — no page-fit / shrink.",
                    ha="center", va="center", fontsize=8, color="grey")

            pdf.savefig(fig)
            plt.close(fig)

    print(f"Wrote {out_path} "
          f"({num_pages} page(s), {cols} × {rows_pp} blocks per page)")


# -----------------------------------------------------------------------
# Radial-gradient SVG (transparent background, gamma-corrected stops)
# -----------------------------------------------------------------------
def render_radial_gradient_svg(rows, out_path, n_exp,
                               diameter_mm=280.0, pad_mm=10.0):
    """Emit an SVG <radialGradient> whose colour stops follow the
    gamma-corrected shade values.

    * Background : transparent (no <rect> fill behind the circle)
    * Centre     : black  (L = 0 %, shade 0)
    * Edge       : white  (L = 100 %, shade N-1)
    * Stops      : at offset i/(N-1) with sRGB grey = L_gamma[i] * 255/100

    Consequence: a 3Pi+ line sensor driven radially through this
    map reads raw ADC values that are linearly spaced in radius,
    because the gamma pre-distortion cancels the n ≈ 2.84 power-law
    response of the phototransistor + printed reflectance.
    """
    n = len(rows)
    size_mm = diameter_mm + 2.0 * pad_mm
    cx = cy = size_mm / 2.0
    r  = diameter_mm / 2.0

    stop_lines = []
    for i, row in enumerate(rows):
        offset = (i / (n - 1)) if n > 1 else 0.0
        g8     = int(round(255.0 * row["L_gamma_%"] / 100.0))
        stop_lines.append(
            f'      <stop offset="{offset:.6f}" '
            f'stop-color="rgb({g8},{g8},{g8})"/>'
        )
    stops_xml = "\n".join(stop_lines)

    svg = (
        '<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n'
        '<svg xmlns="http://www.w3.org/2000/svg" '
        'xmlns:xlink="http://www.w3.org/1999/xlink" version="1.1"\n'
        f'     width="{size_mm}mm" height="{size_mm}mm"\n'
        f'     viewBox="0 0 {size_mm} {size_mm}">\n'
        f'  <title>Gamma-corrected radial gradient (N={n}, n={n_exp:.4f})</title>\n'
        '  <desc>Stops precomputed by gamma_corrected_gradient.py from the\n'
        '        power-law sensor fit in Calibration.h so a 3Pi+ line sensor\n'
        '        driven radially sees a linear raw-ADC response.</desc>\n'
        '  <defs>\n'
        f'    <radialGradient id="gammaRadial"\n'
        f'                    cx="{cx}" cy="{cy}" r="{r}"\n'
        f'                    fx="{cx}" fy="{cy}"\n'
        '                    gradientUnits="userSpaceOnUse"\n'
        '                    spreadMethod="pad">\n'
        f'{stops_xml}\n'
        '    </radialGradient>\n'
        '  </defs>\n'
        f'  <circle cx="{cx}" cy="{cy}" r="{r}" fill="url(#gammaRadial)"/>\n'
        '</svg>\n'
    )
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(svg)
    print(f"Wrote {out_path} "
          f"(diameter {diameter_mm:.0f} mm, {n} stops, transparent background)")


# -----------------------------------------------------------------------
# Comparison PDF (uniform vs gamma strips stacked)
# -----------------------------------------------------------------------
def render_comparison_pdf(rows, out_path, n_exp):
    """Visualisation-only A/B comparison on a single A3 landscape
    page.  Blocks here are scaled-to-fit — NOT at print dimensions —
    because this file is for review, not for driving over.  Labels
    remain strictly outside the blocks for consistency."""
    n = len(rows)
    fig = plt.figure(figsize=A3_LANDSCAPE_IN)

    # Two strips — top and bottom thirds of the page.  Explicit
    # positions avoid tight_layout squashing the rectangles.
    strip_defs = [
        (0.04, 0.58, 0.92, 0.24, "L_uniform_%",
            "Uniform spacing in L (current 25 / 26 shades of grey pattern)"),
        (0.04, 0.16, 0.92, 0.24, "L_gamma_%",
            f"Gamma-corrected spacing — L = 100·(i/(N-1))^(1/n),  n = {n_exp:.3f}"),
    ]

    for (x, y, w, h, key, title) in strip_defs:
        ax = fig.add_axes([x, y, w, h])
        for i, r in enumerate(rows):
            g = r[key] / 100.0
            ax.add_patch(Rectangle((i, 0), 1, 1, facecolor=(g, g, g), edgecolor="none"))
            if n <= 16 or (i == 0 or i == n - 1 or i % max(1, n // 10) == 0):
                ax.text(i + 0.5, 1.04, f"{i}",
                        ha="center", va="bottom", fontsize=9, color="black")
                ax.text(i + 0.5, -0.04, f"{r[key]:.0f}%",
                        ha="center", va="top", fontsize=9, color="black")
        ax.set_xlim(0, n)
        ax.set_ylim(-0.25, 1.25)
        ax.set_aspect("auto")
        ax.axis("off")
        ax.set_title(title, fontsize=11, pad=18)

    # A3 landscape is set by the figsize above; no papertype kwarg
    # needed (and it was removed from modern matplotlib).
    plt.savefig(out_path, format="pdf")
    plt.close(fig)
    print(f"Wrote {out_path}")


# -----------------------------------------------------------------------
# Summary plot (expected ADC vs shade index, uniform vs gamma)
# -----------------------------------------------------------------------
def render_adc_plot(rows, out_path, n_exp, per_sensor_n):
    """Show that the gamma plan produces a linear ADC response."""
    n = len(rows)
    i_axis = np.arange(n)
    # Normalised raw =  A - (L/100)^n   (fractional, no CAL_A/B needed)
    # The ADC step count is proportional; the *shape* vs shade index is
    # what we care about here.
    f_uniform = np.array([r["L_uniform_%"] / 100.0 for r in rows])
    f_gamma   = np.array([r["L_gamma_%"]   / 100.0 for r in rows])

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(i_axis, 1 - f_uniform ** n_exp, "o-",
            label=f"uniform-L plan  (concave, non-linear)",
            color="tab:red")
    ax.plot(i_axis, 1 - f_gamma ** n_exp, "s-",
            label="gamma-corrected plan  (linear in shade i)",
            color="tab:green")
    if per_sensor_n is not None:
        for k, nk in enumerate(per_sensor_n):
            ax.plot(i_axis, 1 - f_gamma ** nk, ":",
                    linewidth=0.9, alpha=0.6,
                    label=f"sensor {k} (n={nk:.3f})")
    ax.set_xlabel("Shade index i")
    ax.set_ylabel("Normalised ADC excursion from black  ( = (L/100)^n )")
    ax.set_title("Predicted sensor response — uniform vs gamma-corrected gradient")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="lower right")
    plt.tight_layout()
    plt.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"Wrote {out_path}")


# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--shades", type=int, default=DEFAULT_SHADES,
                    help=f"number of shades to generate (default {DEFAULT_SHADES})")
    ap.add_argument("--n", type=float, default=None,
                    help="override the power-law exponent (default: mean CAL_N from "
                         f"{DEFAULT_CAL_PATH})")
    ap.add_argument("--cal", default=DEFAULT_CAL_PATH,
                    help="path to Calibration.h (used to source CAL_N if --n not given)")
    ap.add_argument("--out-dir", default=DEFAULT_OUT_DIR,
                    help=f"output directory (default {DEFAULT_OUT_DIR})")
    ap.add_argument("--stem", default=None,
                    help="filename stem for outputs (default: <N>shades_gamma)")
    ap.add_argument("--no-pdf", action="store_true",
                    help="skip PDF outputs, emit CSV + summary plot only")
    ap.add_argument("--no-svg", action="store_true",
                    help="skip the radial-gradient SVG")
    ap.add_argument("--diameter-mm", type=float, default=280.0,
                    help="diameter of the SVG radial gradient (default 280 mm — fits A3 with margin)")
    args = ap.parse_args()

    mean_n, per_sensor_n = read_exponent_from_header(args.cal)
    if args.n is not None:
        n_exp = args.n
        source = "user override"
    elif mean_n is not None:
        n_exp = mean_n
        source = f"mean of CAL_N in {args.cal}"
    else:
        sys.exit(f"ERROR: no --n given and could not read CAL_N from {args.cal}. "
                 f"Run transfer_function.py first or pass --n explicitly.")

    stem = args.stem or f"{args.shades}shades_gamma"
    os.makedirs(args.out_dir, exist_ok=True)

    print(f"Shades       : {args.shades}")
    print(f"Exponent n   : {n_exp:.4f}   ({source})")
    if per_sensor_n:
        print(f"Per-sensor n : {per_sensor_n}")

    rows = compute_shade_table(args.shades, n_exp)

    # Always emit the CSV and the ADC-vs-shade summary plot
    write_csv(rows, os.path.join(args.out_dir, f"{stem}.csv"),
              n_exp, args.shades)
    render_adc_plot(rows, os.path.join(args.out_dir, f"{stem}_adc_preview.png"),
                    n_exp, per_sensor_n)

    if not args.no_pdf:
        render_gradient_pdf(
            rows,
            os.path.join(args.out_dir, f"{stem}_gradient.pdf"),
            f"{args.shades} shades — gamma-corrected (n = {n_exp:.3f})",
            key="L_gamma_%",
        )
        render_comparison_pdf(
            rows,
            os.path.join(args.out_dir, f"{stem}_uniform_vs_gamma.pdf"),
            n_exp,
        )

    if not args.no_svg:
        render_radial_gradient_svg(
            rows,
            os.path.join(args.out_dir, f"{stem}_radial.svg"),
            n_exp,
            diameter_mm=args.diameter_mm,
        )

    # Print a short, human-readable preview
    print("\n i   L_uniform  L_gamma   sRGB(hex)")
    step = max(1, args.shades // 10)
    for r in rows[::step] + ([rows[-1]] if (args.shades - 1) % step else []):
        print(f"{r['shade']:>2}   "
              f"{r['L_uniform_%']:>7.2f}   "
              f"{r['L_gamma_%']:>7.2f}   "
              f"{r['sRGB_hex']}")


if __name__ == "__main__":
    main()
