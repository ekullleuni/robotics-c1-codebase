"""
Optimised Radial Gradient SVG Generator
=========================================
Takes the output of the gradient optimiser and produces an Inkscape-ready
SVG file with colour stops placed at the exact radii computed by the
optimisation.

Inkscape's GUI assumes evenly-spaced stops, so we bypass it entirely and
write the SVG <radialGradient> definition directly.  Open the output in
Inkscape for final tweaks, scaling, or PDF export for printing.

The SVG contains:
  - A radialGradient element with one stop per shade, positioned at the
    optimised normalised radius (0.0 = centre, 1.0 = edge)
  - A circle filled with that gradient at the physical size you specify
  - A thin registration circle at the edge for cutting/alignment
  - A centre crosshair for alignment
  - Inkscape-compatible metadata so it opens cleanly

Usage: run after gradient_optimiser.py, or import and call generate_svg()
directly with the opt dict returned by optimise_gradient().
"""

import numpy as np
from pathlib import Path
import xml.etree.ElementTree as ET


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def shade_to_grey(shade_index: int, n_shades: int) -> int:
    """
    Convert shade index to an 8-bit grey value.
    Shade 0 = black (centre) = 0
    Shade n-1 = white (edge) = 255
    """
    return int(round(shade_index / (n_shades - 1) * 255))


def grey_to_hex(grey: int) -> str:
    return f"#{grey:02x}{grey:02x}{grey:02x}"


# ---------------------------------------------------------------------------
# SVG generation
# ---------------------------------------------------------------------------


def generate_svg(
    opt: dict,
    n_shades: int,
    output_path: str | Path,
    *,
    print_diameter_mm: float
    | None = None,  # physical print size; None = use gradient radius
    svg_px_per_mm: float = 3.7795,  # Inkscape default: 96 dpi -> 3.7795 px/mm
    add_crosshair: bool = True,
    add_registration: bool = True,
    add_annotations: bool = True,
) -> Path:
    """
    Write an Inkscape-ready SVG with the optimised radial gradient.

    Parameters
    ----------
    opt               : dict returned by optimise_gradient()
    n_shades          : number of shade steps (typically 25)
    output_path       : where to write the .svg file
    print_diameter_mm : total circle diameter to draw (defaults to 2× gradient radius)
    svg_px_per_mm     : conversion factor (96 dpi Inkscape default)
    add_crosshair     : draw centre + alignment lines
    add_registration  : draw outer registration circle
    add_annotations   : add text labels for key stop positions
    """
    radius_mm = opt["radius_mm"]
    if print_diameter_mm is None:
        print_diameter_mm = radius_mm * 2.0

    # We draw the gradient circle at exactly gradient_radius_mm,
    # and the overall canvas is print_diameter_mm × print_diameter_mm
    canvas_mm = print_diameter_mm + 20  # 10 mm margin each side
    cx_mm = canvas_mm / 2.0
    cy_mm = canvas_mm / 2.0
    r_px = radius_mm * svg_px_per_mm
    cx_px = cx_mm * svg_px_per_mm
    cy_px = cy_mm * svg_px_per_mm
    canvas_px = canvas_mm * svg_px_per_mm

    # Normalised stop positions (0.0 to 1.0)
    stop_positions = opt["shade_radii"] / radius_mm  # already 0..1

    # ------------------------------------------------------------------
    # Build SVG tree
    # ------------------------------------------------------------------

    # Namespaces Inkscape needs
    nsmap = {
        "xmlns": "http://www.w3.org/2000/svg",
        "xmlns:xlink": "http://www.w3.org/1999/xlink",
        "xmlns:inkscape": "http://www.inkscape.org/namespaces/inkscape",
        "xmlns:sodipodi": "http://sodipodi.sourceforge.net/DTD/sodipodi-0.0.dtd",
        "xmlns:dc": "http://purl.org/dc/elements/1.1/",
        "xmlns:cc": "http://creativecommons.org/ns#",
        "xmlns:rdf": "http://www.w3.org/1999/02/22-rdf-syntax-ns#",
    }

    svg_attribs = {
        **nsmap,
        "width": f"{canvas_px:.4f}",
        "height": f"{canvas_px:.4f}",
        "viewBox": f"0 0 {canvas_px:.4f} {canvas_px:.4f}",
        "version": "1.1",
        "id": "svg_gradient",
        "inkscape:document-units": "mm",
    }

    svg = ET.Element("svg", svg_attribs)

    # -- Inkscape page metadata (sodipodi:namedview) ----------------------
    nv = ET.SubElement(
        svg,
        "sodipodi:namedview",
        {
            "id": "namedview1",
            "pagecolor": "#ffffff",
            "bordercolor": "#666666",
            "borderopacity": "1.0",
            "inkscape:pageopacity": "0.0",
            "inkscape:pageshadow": "2",
            "inkscape:document-units": "mm",
            "showgrid": "false",
            "units": "mm",
        },
    )

    # -- defs: radial gradient -------------------------------------------
    defs = ET.SubElement(svg, "defs")

    rg = ET.SubElement(
        defs,
        "radialGradient",
        {
            "id": "optimisedGradient",
            "cx": "50%",
            "cy": "50%",
            "r": "50%",
            "fx": "50%",
            "fy": "50%",
            "gradientUnits": "objectBoundingBox",
        },
    )

    for i in range(n_shades):
        grey = shade_to_grey(i, n_shades)
        hex_col = grey_to_hex(grey)
        pos = stop_positions[i]
        stop = ET.SubElement(
            rg,
            "stop",
            {
                "offset": f"{pos:.6f}",
                "style": f"stop-color:{hex_col};stop-opacity:1",
                "id": f"stop_{i}",
            },
        )
        # Add inkscape label for easy identification
        stop.set("inkscape:label", f"shade_{i}_r{opt['shade_radii'][i]:.1f}mm")

    # Beyond the last stop (r > gradient radius but inside canvas): stay white
    ET.SubElement(
        rg,
        "stop",
        {
            "offset": "1.0",
            "style": "stop-color:#ffffff;stop-opacity:1",
            "id": "stop_edge",
        },
    )

    # -- Main gradient layer ---------------------------------------------
    layer_gradient = ET.SubElement(
        svg,
        "g",
        {
            "id": "layer_gradient",
            "inkscape:label": "Gradient",
            "inkscape:groupmode": "layer",
        },
    )

    # Gradient circle
    ET.SubElement(
        layer_gradient,
        "circle",
        {
            "id": "gradient_circle",
            "cx": f"{cx_px:.4f}",
            "cy": f"{cy_px:.4f}",
            "r": f"{r_px:.4f}",
            "style": "fill:url(#optimisedGradient);stroke:none",
            "inkscape:label": f"Gradient circle r={radius_mm:.1f}mm",
        },
    )

    # -- Registration / alignment layer ----------------------------------
    if add_registration or add_crosshair:
        layer_reg = ET.SubElement(
            svg,
            "g",
            {
                "id": "layer_registration",
                "inkscape:label": "Registration",
                "inkscape:groupmode": "layer",
                "style": "display:inline",
            },
        )

        reg_style = "fill:none;stroke:#ff0000;stroke-width:0.5;stroke-dasharray:none"

        if add_registration:
            # Outer registration circle (hairline, for cutting guide)
            ET.SubElement(
                layer_reg,
                "circle",
                {
                    "id": "registration_circle",
                    "cx": f"{cx_px:.4f}",
                    "cy": f"{cy_px:.4f}",
                    "r": f"{r_px:.4f}",
                    "style": reg_style,
                    "inkscape:label": "Cut guide",
                },
            )

        if add_crosshair:
            cross_len_px = 8 * svg_px_per_mm  # 8 mm arms
            # Horizontal bar
            ET.SubElement(
                layer_reg,
                "line",
                {
                    "id": "crosshair_h",
                    "x1": f"{cx_px - cross_len_px:.4f}",
                    "y1": f"{cy_px:.4f}",
                    "x2": f"{cx_px + cross_len_px:.4f}",
                    "y2": f"{cy_px:.4f}",
                    "style": reg_style,
                },
            )
            # Vertical bar
            ET.SubElement(
                layer_reg,
                "line",
                {
                    "id": "crosshair_v",
                    "x1": f"{cx_px:.4f}",
                    "y1": f"{cy_px - cross_len_px:.4f}",
                    "x2": f"{cx_px:.4f}",
                    "y2": f"{cy_px + cross_len_px:.4f}",
                    "style": reg_style,
                },
            )
            # Centre dot
            ET.SubElement(
                layer_reg,
                "circle",
                {
                    "id": "centre_dot",
                    "cx": f"{cx_px:.4f}",
                    "cy": f"{cy_px:.4f}",
                    "r": f"{0.5 * svg_px_per_mm:.4f}",
                    "style": "fill:#ff0000;stroke:none",
                },
            )

    # -- Annotation layer ------------------------------------------------
    if add_annotations:
        layer_ann = ET.SubElement(
            svg,
            "g",
            {
                "id": "layer_annotations",
                "inkscape:label": "Annotations",
                "inkscape:groupmode": "layer",
                "style": "display:inline",
            },
        )

        ann_style = "font-family:monospace;font-size:7px;fill:#0000cc;stroke:none"

        # Title block
        ET.SubElement(
            layer_ann,
            "text",
            {
                "x": f"{10 * svg_px_per_mm:.4f}",
                "y": f"{8 * svg_px_per_mm:.4f}",
                "style": "font-family:monospace;font-size:9px;font-weight:bold;fill:#000000",
            },
        ).text = f"3pi+ Optimised Radial Gradient  |  r={radius_mm:.0f}mm  |  {n_shades} shades"

        ET.SubElement(
            layer_ann,
            "text",
            {
                "x": f"{10 * svg_px_per_mm:.4f}",
                "y": f"{13 * svg_px_per_mm:.4f}",
                "style": ann_style,
            },
        ).text = "Shade 0=black (centre)  →  Shade 24=white (edge)  |  stops at optimised radii"

        # Annotate every 4th stop with a radial tick + label
        for i in range(0, n_shades, 4):
            r_stop_px = stop_positions[i] * r_px
            # Point on the right side of the circle
            tick_x1 = cx_px + r_stop_px
            tick_x2 = cx_px + r_stop_px + 4 * svg_px_per_mm
            tick_y = cy_px
            ET.SubElement(
                layer_ann,
                "line",
                {
                    "x1": f"{tick_x1:.4f}",
                    "y1": f"{tick_y:.4f}",
                    "x2": f"{tick_x2:.4f}",
                    "y2": f"{tick_y:.4f}",
                    "style": "stroke:#0000cc;stroke-width:0.4",
                },
            )
            ET.SubElement(
                layer_ann,
                "text",
                {
                    "x": f"{tick_x2 + svg_px_per_mm:.4f}",
                    "y": f"{tick_y + 2 * svg_px_per_mm:.4f}",
                    "style": ann_style,
                },
            ).text = f"s{i} {opt['shade_radii'][i]:.1f}mm"

    # -- Write -----------------------------------------------------------
    output_path = Path(output_path)
    tree = ET.ElementTree(svg)
    ET.indent(tree, space="  ")

    with open(output_path, "wb") as f:
        f.write(b'<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n')
        tree.write(f, encoding="utf-8", xml_declaration=False)

    return output_path


# ---------------------------------------------------------------------------
# Standalone demo — uses hardcoded example opt dict if run directly
# (in practice, import and call generate_svg() from gradient_optimiser.py)
# ---------------------------------------------------------------------------


def _demo():
    """
    Generates a demo SVG with a hypothetical optimisation result so you can
    verify the output in Inkscape without needing the full data pipeline.
    Shade radii are linearly spaced here — run gradient_optimiser.py to get
    the real non-uniform spacing.
    """
    n_shades = 25
    radius_mm = 100.0
    shade_radii = np.linspace(0, radius_mm, n_shades)

    opt_demo = {
        "shade_radii": shade_radii,
        "pair_widths_mm": np.diff(shade_radii),
        "radius_mm": radius_mm,
    }

    out = Path("gradient_demo.svg")
    generate_svg(opt_demo, n_shades, out, print_diameter_mm=220.0)
    print(f"Demo SVG written to {out}")
    print("Open in Inkscape, then File > Export PNG or File > Print.")


if __name__ == "__main__":
    _demo()
