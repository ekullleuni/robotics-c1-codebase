"""
Radial Gradient Optimiser for 3pi+ Line Sensor Navigation
===========================================================
Uses reflectance sensor resolution data (from shade calibration) to design an
optimal radial gradient for centre-finding tasks.

PROBLEM:
  A radial gradient runs from black (centre) to white (edge).  The robot must
  infer its radial position from a single sensor reading.  If the gradient
  maps physical radius linearly to shade, the robot's effective positional
  resolution is non-uniform — it is high where the sensor distinguishes shades
  easily, and low (or zero) where consecutive shades overlap.

APPROACH:
  1. Compute per-sensor "local distinguishability" across the 25-shade palette.
  2. Derive an optimal shade-to-radius mapping that equalises positional
     resolution: stretch the gradient (more physical space per shade step)
     where sensor discrimination is poor, compress it where discrimination
     is good.
  3. Output a printable gradient profile and a recommended radius-to-shade
     lookup for firmware use.

Data format expected: shade_id, sample, timestamp_ms, s0, s1, s2, s3, s4
Shade 0 = darkest (centre), shade 24 = lightest (edge).
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path

# ---------------------------------------------------------------------------
# Re-use core loader from the resolution analysis script
# (copy the relevant pieces here so this script is self-contained)
# ---------------------------------------------------------------------------

SENSORS = ["s0", "s1", "s2", "s3", "s4"]
SETTLE_SAMPLES = 7


def load_shade_data(filepath: str) -> dict:
    raw: dict[int, dict[str, list]] = {}
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) != 8:
                continue
            shade_id = int(parts[0])
            if shade_id not in raw:
                raw[shade_id] = {s: [] for s in SENSORS}
            for j, s in enumerate(SENSORS):
                raw[shade_id][s].append(int(parts[3 + j]))
    data = {}
    for shade_id in sorted(raw.keys()):
        data[shade_id] = {s: np.array(raw[shade_id][s]) for s in SENSORS}
    return data


def combine_datasets(filepaths: list) -> dict:
    combined: dict[int, dict[str, list]] = {}
    for fp in filepaths:
        data = load_shade_data(fp)
        for shade_id in data:
            if shade_id not in combined:
                combined[shade_id] = {s: [] for s in SENSORS}
            for s in SENSORS:
                stable = data[shade_id][s][SETTLE_SAMPLES:]
                combined[shade_id][s].extend(stable.tolist())
    result = {}
    for shade_id in sorted(combined.keys()):
        result[shade_id] = {s: np.array(combined[shade_id][s]) for s in SENSORS}
    return result


def compute_shade_stats(data: dict):
    means = {s: [] for s in SENSORS}
    mins = {s: [] for s in SENSORS}
    maxs = {s: [] for s in SENSORS}
    counts = []
    for shade_id in sorted(data.keys()):
        counts.append(len(data[shade_id][SENSORS[0]]))
        for s in SENSORS:
            arr = data[shade_id][s]
            means[s].append(np.mean(arr))
            mins[s].append(int(np.min(arr)))
            maxs[s].append(int(np.max(arr)))
    for s in SENSORS:
        means[s] = np.array(means[s])
        mins[s] = np.array(mins[s])
        maxs[s] = np.array(maxs[s])
    return means, mins, maxs, counts


# ---------------------------------------------------------------------------
# Gradient optimisation core
# ---------------------------------------------------------------------------


def compute_worst_case_gaps(mins: dict, maxs: dict, n_shades: int) -> dict:
    """For consecutive shade pairs return min[i] - max[i+1].
    Positive = safely separable, negative = overlap."""
    gaps = {}
    for s in SENSORS:
        gaps[s] = np.array(
            [int(mins[s][i]) - int(maxs[s][i + 1]) for i in range(n_shades - 1)]
        )
    return gaps


def compute_local_snr(means: dict, mins: dict, maxs: dict, n_shades: int) -> dict:
    """
    Local signal-to-noise ratio for each consecutive shade pair.

    SNR = signal / noise
      signal = |mean[i] - mean[i+1]|        (how far apart the means are)
      noise  = (spread[i] + spread[i+1]) / 2 (average half-range of both)

    SNR > 1  -> means are well-separated relative to noise
    SNR < 1  -> shades likely to be confused
    SNR = 0  -> identical means (degenerate)
    """
    snr = {}
    for s in SENSORS:
        snr_s = []
        for i in range(n_shades - 1):
            signal = abs(float(means[s][i]) - float(means[s][i + 1]))
            spread_i = float(maxs[s][i]) - float(mins[s][i])
            spread_i1 = float(maxs[s][i + 1]) - float(mins[s][i + 1])
            noise = (spread_i + spread_i1) / 2.0
            if noise < 1e-6:
                snr_s.append(0.0 if signal < 1e-6 else float("inf"))
            else:
                snr_s.append(signal / noise)
        snr[s] = np.array(snr_s)
    return snr


def compute_distinguishability_score(
    gaps: dict,
    snr: dict,
    n_shades: int,
    gap_weight: float = 0.5,
    snr_weight: float = 0.5,
) -> dict:
    """
    Combine worst-case gap and SNR into a single per-pair distinguishability
    score in [0, 1], where 1 = perfectly distinguishable.

    gap score: sigmoid of the worst-case gap (normalised by typical spread)
    snr score: min(SNR, 4) / 4  (saturates above SNR=4; diminishing returns)
    """
    scores = {}
    # Normalise gaps across all sensors to get a sensor-independent scale
    all_gaps = np.concatenate([gaps[s] for s in SENSORS])
    gap_scale = max(np.std(all_gaps), 1.0)

    def sigmoid(x):
        return 1.0 / (1.0 + np.exp(-x))

    for s in SENSORS:
        gap_score = sigmoid(gaps[s] / gap_scale)  # 0..1
        snr_score = np.clip(snr[s] / 4.0, 0.0, 1.0)  # 0..1
        scores[s] = gap_weight * gap_score + snr_weight * snr_score
    return scores


def optimise_gradient(
    scores: dict,
    n_shades: int,
    primary_sensor: str = "s2",
    radius_mm: float = 100.0,
    blend_sensors: bool = True,
) -> dict:
    """
    Compute the optimal shade-to-radius mapping.

    The idea: give MORE physical radius (mm) to shade transitions where the
    sensor struggles (low score), and LESS physical radius where it excels
    (high score).  This equalises positional resolution across the gradient.

    Steps:
      1. Compute a "difficulty weight" = 1 / distinguishability_score for each
         consecutive pair.  Harder pairs get proportionally more radius.
      2. Normalise so the weights sum to 1, then multiply by total radius.
      3. Integrate to get the radius at each shade boundary.

    If blend_sensors=True, weights are averaged across sensors before
    optimisation (produces a gradient robust to which sensor is active).

    Returns a dict with:
      shade_radii    : radius (mm) at each shade index (0..n_shades-1)
      pair_widths_mm : physical width allocated to each shade transition
      difficulty     : per-pair difficulty weights (pre-normalisation)
      linear_radii   : linear reference (for comparison)
    """
    if blend_sensors:
        # Mean across sensors — robust to sensor position over gradient
        combined = np.mean(np.stack([scores[s] for s in SENSORS], axis=0), axis=0)
    else:
        combined = scores[primary_sensor]

    # Width is PROPORTIONAL to distinguishability:
    #   high score (easy pair) -> more mm per shade step (shallow, stretched)
    #   low score (hard pair)  -> less mm per shade step (steep, compressed)
    # This forces a faster rate of intensity change where the sensor struggles,
    # giving it a stronger signal per mm to compensate for poor discrimination.
    width_weights = np.clip(combined, 0.05, 1.0)

    # Normalise so total sums to radius_mm
    difficulty_norm = width_weights / width_weights.sum()

    # Pair widths in mm
    pair_widths_mm = difficulty_norm * radius_mm

    # Keep difficulty as the inverse for reporting (conceptually clearer)
    difficulty = 1.0 / width_weights

    # Shade radii: shade 0 starts at r=0, each subsequent shade is further out
    shade_radii = np.zeros(n_shades)
    for i in range(1, n_shades):
        shade_radii[i] = shade_radii[i - 1] + pair_widths_mm[i - 1]

    linear_radii = np.linspace(0, radius_mm, n_shades)

    return {
        "shade_radii": shade_radii,
        "pair_widths_mm": pair_widths_mm,
        "difficulty": difficulty,
        "difficulty_norm": difficulty_norm,
        "linear_radii": linear_radii,
        "radius_mm": radius_mm,
    }


def compute_uniform_resolution(
    opt_result: dict,
    means: dict,
    mins: dict,
    maxs: dict,
    n_shades: int,
    sensor: str = "s2",
) -> dict:
    """
    Estimate effective positional resolution (mm per ADC count) at each
    shade under the optimised gradient vs a linear gradient.

    resolution = pair_width_mm / |mean[i] - mean[i+1]|
    Lower is better (more counts per mm = finer position sensing).
    """
    resolutions_opt = []
    resolutions_linear = []
    linear_width = opt_result["radius_mm"] / (n_shades - 1)

    for i in range(n_shades - 1):
        delta_mean = abs(float(means[sensor][i]) - float(means[sensor][i + 1]))
        if delta_mean < 1e-3:
            delta_mean = 1e-3  # avoid division by zero

        resolutions_opt.append(opt_result["pair_widths_mm"][i] / delta_mean)
        resolutions_linear.append(linear_width / delta_mean)

    return {
        "opt": np.array(resolutions_opt),
        "linear": np.array(resolutions_linear),
    }


# ---------------------------------------------------------------------------
# Printing
# ---------------------------------------------------------------------------


def print_gradient_report(
    opt: dict,
    scores: dict,
    gaps: dict,
    resolutions: dict,
    n_shades: int,
    primary_sensor: str = "s2",
):
    sep = "=" * 80
    print(sep)
    print("RADIAL GRADIENT OPTIMISATION REPORT")
    print(sep)
    print(f"\nTotal gradient radius : {opt['radius_mm']:.1f} mm")
    print(f"Number of shades      : {n_shades}")
    print(f"Primary sensor        : {primary_sensor}")
    print(f"\nShade 0  → black  (centre of gradient, r = 0 mm)")
    print(
        f"Shade {n_shades - 1:2d} → white  (edge   of gradient, r = {opt['radius_mm']:.1f} mm)"
    )

    print(f"\n{sep}")
    print("SHADE BOUNDARY RADII")
    print(
        f"{'Shade':>5} | {'Linear r (mm)':>14} | {'Optimised r (mm)':>16} | "
        f"{'Δ from linear':>13} | {'Pair width (mm)':>14} | {'Difficulty':>10}"
    )
    print("-" * 82)
    for i in range(n_shades):
        width_str = f"{opt['pair_widths_mm'][i - 1]:.2f}" if i > 0 else "—"
        diff_str = f"{opt['difficulty'][i - 1]:.3f}" if i > 0 else "—"
        delta = opt["shade_radii"][i] - opt["linear_radii"][i]
        print(
            f"{i:>5} | {opt['linear_radii'][i]:>14.2f} | "
            f"{opt['shade_radii'][i]:>16.2f} | {delta:>+13.2f} | "
            f"{width_str:>14} | {diff_str:>10}"
        )

    print(f"\n{sep}")
    print("WHERE TO STRETCH vs COMPRESS THE GRADIENT")
    print(sep)
    print(
        f"\n  Pair widths > {opt['radius_mm'] / (n_shades - 1):.2f} mm = STRETCHED  (sensor excels here → can afford shallow gradient)"
    )
    print(
        f"  Pair widths < {opt['radius_mm'] / (n_shades - 1):.2f} mm = COMPRESSED (sensor struggles → steeper gradient forces stronger signal change per mm)\n"
    )

    linear_width = opt["radius_mm"] / (n_shades - 1)
    for i in range(n_shades - 1):
        w = opt["pair_widths_mm"][i]
        gap = gaps[primary_sensor][i]
        action = (
            "STRETCH  ↔"
            if w > linear_width * 1.05
            else ("COMPRESS ↕" if w < linear_width * 0.95 else "≈ linear  ")
        )
        print(
            f"  Shade {i:>2}→{i + 1:>2} : {w:>6.2f} mm  {action}  "
            f"(worst-case gap={gap:+5d}, score={scores[primary_sensor][i]:.3f})"
        )

    print(f"\n{sep}")
    print("POSITIONAL RESOLUTION COMPARISON (sensor s2)")
    print(sep)
    print(f"\n  mm per ADC count — lower = finer position sensing")
    print(
        f"  {'Pair':>6} | {'Linear (mm/count)':>18} | {'Optimised (mm/count)':>20} | {'Improvement':>12}"
    )
    print("  " + "-" * 65)
    for i in range(n_shades - 1):
        imp = resolutions["linear"][i] / resolutions["opt"][i]
        flag = " ✓" if imp > 1.05 else (" ✗" if imp < 0.95 else "  ")
        print(
            f"  {i:>2}→{i + 1:>2}  | {resolutions['linear'][i]:>18.4f} | "
            f"{resolutions['opt'][i]:>20.4f} | {imp:>10.2f}×{flag}"
        )

    print(f"\n{sep}")
    print("FIRMWARE LOOKUP TABLE  (shade index → radius in mm, integer mm)")
    print(sep)
    print("\n  // Copy into your 3pi+ firmware:")
    print("  const float SHADE_RADIUS_MM[] = {", end="")
    vals = [f"{r:.1f}" for r in opt["shade_radii"]]
    print(", ".join(vals) + "};")
    print(f"  // shade 0 = black (centre), shade {n_shades - 1} = white (edge)")
    print(f"  // gradient radius = {opt['radius_mm']:.0f} mm\n")


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

SENSOR_COLOURS = {
    "s0": "#534AB7",
    "s1": "#1D9E75",
    "s2": "#D85A30",
    "s3": "#3266ad",
    "s4": "#888780",
}


def plot_gradient_analysis(
    opt: dict,
    scores: dict,
    gaps: dict,
    resolutions: dict,
    means: dict,
    n_shades: int,
    n_files: int,
    primary_sensor: str = "s2",
):
    shade_idx = np.arange(n_shades)
    pair_idx = np.arange(n_shades - 1)

    fig = plt.figure(figsize=(15, 20))
    gs = gridspec.GridSpec(
        4, 2, hspace=0.40, wspace=0.32, left=0.08, right=0.95, top=0.96, bottom=0.04
    )

    suptitle = "Radial Gradient Optimisation"
    if n_files > 1:
        suptitle += f"  ({n_files} calibration sessions)"
    fig.suptitle(suptitle, fontsize=14, fontweight="bold", y=0.99)

    linear_width = opt["radius_mm"] / (n_shades - 1)

    # ------------------------------------------------------------------
    # Panel 1: Distinguishability scores per sensor (full width)
    # ------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, :])
    for s in SENSORS:
        ax1.plot(
            pair_idx,
            scores[s],
            "o-",
            color=SENSOR_COLOURS[s],
            markersize=4,
            linewidth=1.5,
            label=s,
            alpha=0.85,
        )
    ax1.axhline(
        0.5,
        color="black",
        linewidth=0.8,
        linestyle="--",
        alpha=0.4,
        label="threshold (0.5)",
    )
    ax1.set_xlabel("Consecutive shade pair (i → i+1)")
    ax1.set_ylabel("Distinguishability score (0=worst, 1=best)")
    ax1.set_title(
        "Per-pair sensor distinguishability  (combined gap + SNR score)",
        fontweight="bold",
    )
    ax1.legend(loc="lower right", fontsize=8, framealpha=0.9)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(-0.5, n_shades - 1.5)
    ax1.set_ylim(0, 1.05)

    # ------------------------------------------------------------------
    # Panel 2: Optimised pair widths vs linear (full width)
    # ------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[1, :])
    colours = [
        "#E24B4A"
        if w > linear_width * 1.05
        else ("#1D9E75" if w < linear_width * 0.95 else "#aaaaaa")
        for w in opt["pair_widths_mm"]
    ]
    bars = ax2.bar(
        pair_idx,
        opt["pair_widths_mm"],
        color=colours,
        alpha=0.85,
        label="Optimised width",
    )
    ax2.axhline(
        linear_width,
        color="black",
        linewidth=1.5,
        linestyle="--",
        alpha=0.7,
        label=f"Linear reference ({linear_width:.2f} mm)",
    )
    ax2.set_xlabel("Shade pair (i → i+1)  [shade 0=black, 24=white]")
    ax2.set_ylabel("Physical width in gradient (mm)")
    ax2.set_title(
        "Optimised shade-pair widths  "
        "(red = compress / steeper gradient, green = stretch / shallower, grey ≈ linear)",
        fontweight="bold",
    )
    ax2.legend(loc="upper right", fontsize=9, framealpha=0.9)
    ax2.grid(True, axis="y", alpha=0.3)
    ax2.set_xlim(-0.5, n_shades - 1.5)

    # Annotate large deviations
    for i, (bar, w) in enumerate(zip(bars, opt["pair_widths_mm"])):
        if abs(w - linear_width) > linear_width * 0.4:
            ax2.text(
                bar.get_x() + bar.get_width() / 2,
                w + 0.1,
                f"{w:.1f}",
                ha="center",
                va="bottom",
                fontsize=7,
            )

    # ------------------------------------------------------------------
    # Panel 3: Linear vs optimised radius mapping (mid-left)
    # ------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[2, 0])
    ax3.plot(
        shade_idx,
        opt["linear_radii"],
        "k--",
        linewidth=1.5,
        label="Linear mapping",
        alpha=0.7,
    )
    ax3.plot(
        shade_idx,
        opt["shade_radii"],
        "o-",
        color="#D85A30",
        linewidth=2,
        markersize=5,
        label="Optimised mapping",
    )
    ax3.fill_between(
        shade_idx,
        opt["linear_radii"],
        opt["shade_radii"],
        where=(opt["shade_radii"] > opt["linear_radii"]),
        alpha=0.15,
        color="#E24B4A",
        label="Stretched region",
    )
    ax3.fill_between(
        shade_idx,
        opt["linear_radii"],
        opt["shade_radii"],
        where=(opt["shade_radii"] < opt["linear_radii"]),
        alpha=0.15,
        color="#1D9E75",
        label="Compressed region",
    )
    ax3.set_xlabel("Shade index (0=black / centre, 24=white / edge)")
    ax3.set_ylabel("Radius (mm)")
    ax3.set_title("Shade → Radius mapping\n(optimised vs linear)", fontweight="bold")
    ax3.legend(fontsize=8, framealpha=0.9)
    ax3.grid(True, alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 4: Radial gradient preview strip (mid-right)
    # ------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[2, 1])
    # Build a 1D image where pixel x maps to radius, colour = grey level
    n_pixels = 400
    pixel_radii = np.linspace(0, opt["radius_mm"], n_pixels)
    pixel_grey = np.zeros(n_pixels)
    for px, r in enumerate(pixel_radii):
        # Find which shade this radius falls in
        shade_idx_px = np.searchsorted(opt["shade_radii"], r, side="right") - 1
        shade_idx_px = int(np.clip(shade_idx_px, 0, n_shades - 1))
        # Grey value: shade 0 = black (0.0), shade 24 = white (1.0)
        pixel_grey[px] = shade_idx_px / (n_shades - 1)

    # Also compute linear version
    pixel_grey_linear = np.zeros(n_pixels)
    for px, r in enumerate(pixel_radii):
        shade_idx_lin = np.searchsorted(opt["linear_radii"], r, side="right") - 1
        shade_idx_lin = int(np.clip(shade_idx_lin, 0, n_shades - 1))
        pixel_grey_linear[px] = shade_idx_lin / (n_shades - 1)

    combined_strip = np.vstack(
        [
            np.tile(pixel_grey_linear, (60, 1)),
            np.ones((4, n_pixels)) * 0.5,  # divider line
            np.tile(pixel_grey, (60, 1)),
        ]
    )
    ax4.imshow(
        combined_strip,
        cmap="gray",
        vmin=0,
        vmax=1,
        aspect="auto",
        extent=[0, opt["radius_mm"], 0, 1],
    )
    ax4.set_xlabel("Radius (mm) →  black=centre, white=edge")
    ax4.set_yticks([0.22, 0.78])
    ax4.set_yticklabels(["Optimised", "Linear"], fontsize=9)
    ax4.set_title(
        "Gradient preview strip\n(top = linear, bottom = optimised)", fontweight="bold"
    )

    # ------------------------------------------------------------------
    # Panel 5: Positional resolution comparison (bottom-left)
    # ------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[3, 0])
    ax5.plot(
        pair_idx,
        resolutions["linear"],
        "k--o",
        markersize=4,
        linewidth=1.5,
        label="Linear gradient",
        alpha=0.7,
    )
    ax5.plot(
        pair_idx,
        resolutions["opt"],
        "o-",
        color="#D85A30",
        markersize=4,
        linewidth=1.8,
        label="Optimised gradient",
    )
    ax5.set_xlabel("Shade pair (i → i+1)")
    ax5.set_ylabel("mm per ADC count  (lower = better)")
    ax5.set_title(
        f"Positional resolution by region  ({primary_sensor})\n"
        "lower = finer position sensing",
        fontweight="bold",
    )
    ax5.legend(fontsize=8, framealpha=0.9)
    ax5.grid(True, alpha=0.3)
    ax5.set_xlim(-0.5, n_shades - 1.5)

    # ------------------------------------------------------------------
    # Panel 6: Worst-case gaps coloured by region type (bottom-right)
    # ------------------------------------------------------------------
    ax6 = fig.add_subplot(gs[3, 1])
    gap_vals = gaps[primary_sensor]
    colours6 = ["#E24B4A" if g < 0 else "#1D9E75" for g in gap_vals]
    ax6.bar(pair_idx, gap_vals, color=colours6, alpha=0.85)
    ax6.axhline(0, color="black", linewidth=1, alpha=0.5)
    # Overlay the optimised width as a line (secondary axis)
    ax6b = ax6.twinx()
    ax6b.plot(
        pair_idx,
        opt["pair_widths_mm"],
        "^--",
        color="#D85A30",
        markersize=5,
        linewidth=1.2,
        label="Optimised width (mm)",
        alpha=0.8,
    )
    ax6b.axhline(linear_width, color="#D85A30", linewidth=0.8, linestyle=":", alpha=0.5)
    ax6b.set_ylabel("Optimised pair width (mm)", color="#D85A30")
    ax6b.tick_params(axis="y", labelcolor="#D85A30")
    ax6b.legend(loc="upper right", fontsize=8)
    ax6.set_xlabel("Shade pair (i → i+1)")
    ax6.set_ylabel("Worst-case gap (counts)")
    ax6.set_title(
        f"Gap vs optimised width  ({primary_sensor})\n"
        "confirms: poor-gap pairs get more physical space",
        fontweight="bold",
    )
    ax6.grid(True, axis="y", alpha=0.3)

    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    filepaths = [
        Path("data/25shades/20260313_190654.csv"),
        Path("data/25shades/20260313_191315.csv"),
    ]
    for fp in filepaths:
        if not fp.exists():
            print(f"Error: file not found: {fp}")
            sys.exit(1)

    # ── User-configurable parameters ──────────────────────────────────────
    GRADIENT_RADIUS_MM = 100.0  # total physical radius of printed gradient
    PRIMARY_SENSOR = "s2"  # sensor used for resolution comparison
    BLEND_SENSORS = True  # True = average all sensors for gradient opt
    GAP_WEIGHT = 0.5  # weight of worst-case gap in score (0..1)
    SNR_WEIGHT = 0.5  # weight of SNR in score (0..1)
    # ──────────────────────────────────────────────────────────────────────

    print(f"Loading {len(filepaths)} file(s) ...")
    data = combine_datasets([str(fp) for fp in filepaths])
    n_shades = len(data)

    means, mins, maxs, counts = compute_shade_stats(data)
    gaps = compute_worst_case_gaps(mins, maxs, n_shades)
    snr = compute_local_snr(means, mins, maxs, n_shades)
    scores = compute_distinguishability_score(
        gaps, snr, n_shades, GAP_WEIGHT, SNR_WEIGHT
    )

    opt = optimise_gradient(
        scores, n_shades, PRIMARY_SENSOR, GRADIENT_RADIUS_MM, BLEND_SENSORS
    )

    resolutions = compute_uniform_resolution(
        opt, means, mins, maxs, n_shades, PRIMARY_SENSOR
    )

    print_gradient_report(opt, scores, gaps, resolutions, n_shades, PRIMARY_SENSOR)

    print("Generating plots ...")
    fig = plot_gradient_analysis(
        opt, scores, gaps, resolutions, means, n_shades, len(filepaths), PRIMARY_SENSOR
    )

    out_path = filepaths[0].parent / "gradient_optimisation.png"
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    print(f"Figure saved to {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
