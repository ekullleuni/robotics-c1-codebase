"""
transfer_function.py
--------------------
Fits per-sensor calibration curves for the 3Pi+ line sensors from a
shade-characterisation CSV, writes a ready-to-include `Calibration.h`
Arduino header directly into the GradientCentreFinder sketch folder,
and saves a diagnostic plot.

Two models are fit to every sensor:

  * LINEAR     y = slope*L + intercept        (kept for reference/diagnostics)
  * POWER-LAW  y = a − b·(L/100)^n            (used by the firmware)

The power-law form is the physically-motivated response of the
phototransistor / pull-up / printed-gamma stack (see the companion
report `TransferFunction_Analysis.md`).  Its inverse,

      L = 100 * ((a − raw) / b) ** (1/n)

is implemented on the robot by `gradientToPercent()` in
GradientCalibration.h.

Usage:
    python transfer_function.py                         # most recent Nshades_*.csv
    python transfer_function.py data/25shades_*.csv     # explicit file

Input filename must match the pattern  <N>shades_<stamp>.csv
where <N> is the number of shade levels measured (shade_id 0
= black, shade_id N-1 = white).

Outputs:
    GradientCentreFinder/Calibration.h             (Arduino header —
                                                    consumed by the
                                                    centre-finder sketch)
    data/<input_stem>_transfer_functions.png       (diagnostic plot)
"""

import os
import re
import sys
import csv
import glob
import collections
import datetime

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


# -----------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------
DATA_DIR       = "data"
# Calibration header is written directly into the GradientCentreFinder
# sketch folder so the next  arduino-cli compile  picks it up without
# any manual copy step.  Path is relative to this script's CWD.
CALIB_OUT_PATH = os.path.join("GradientCentreFinder", "Calibration.h")
NUM_SENSORS    = 5
MIN_RANGE_WARN = 50.0   # Sensor must span more than this ADC range (sanity check)


# -----------------------------------------------------------------------
# File resolution
# -----------------------------------------------------------------------
def resolve_input_path(argv):
    """Pick the CSV to analyse: CLI arg if given, else newest Nshades_*.csv."""
    if len(argv) > 1:
        path = argv[1]
        if not os.path.exists(path):
            print(f"ERROR: file not found: {path}")
            sys.exit(1)
        return path

    candidates = sorted(
        glob.glob(os.path.join(DATA_DIR, "*shades_*.csv")),
        key=os.path.getmtime,
    )
    if not candidates:
        print(f"ERROR: no {DATA_DIR}/Nshades_*.csv files found. "
              f"Run log_sensors.py first or pass a path.")
        sys.exit(1)
    return candidates[-1]


def parse_shade_count(filepath):
    """Extract N from '<N>shades_' in the filename."""
    m = re.search(r"(\d+)shades_", os.path.basename(filepath))
    if not m:
        print(f"ERROR: could not parse shade count from filename: {filepath}")
        print("       Expected pattern:  <N>shades_<timestamp>.csv")
        sys.exit(1)
    return int(m.group(1))


# -----------------------------------------------------------------------
# Data loading
# -----------------------------------------------------------------------
def load_data(filepath):
    """Return dict {shade_id: [[s0..s4], ...]}, skipping comments/blank lines."""
    data = collections.defaultdict(list)
    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) != 7:
                continue
            try:
                sid = int(parts[0])
                sensors = [int(x) for x in parts[2:7]]
            except ValueError:
                continue
            data[sid].append(sensors)
    return data


# -----------------------------------------------------------------------
# Calibration fitting
# -----------------------------------------------------------------------
def fit_calibration(data, n_shades):
    """
    For each sensor k, compute:
        means[sid, k]              mean raw ADC at shade sid
        fit[k]   = (slope, intercept)  from lightness(%) -> mean_adc
        r2[k]    coefficient of determination for that fit
        white[k] = mean ADC at shade_id == n_shades-1
        black[k] = mean ADC at shade_id == 0
        range[k] = black[k] - white[k]

    Returns a dict with all of the above plus the lightness axis.
    Gracefully handles missing shade_ids (they are skipped from the fit).
    """
    present_ids = sorted(data.keys())
    if not present_ids:
        print("ERROR: no usable data rows in the CSV.")
        sys.exit(1)

    if len(present_ids) != n_shades:
        missing = sorted(set(range(n_shades)) - set(present_ids))
        print(f"WARNING: filename claims {n_shades} shades but "
              f"{len(present_ids)} unique shade_ids present. "
              f"Missing ids: {missing}")

    # Lightness is 0% for shade 0 (black) -> 100% for shade n_shades-1 (white)
    lightness_all = np.array([i * 100.0 / (n_shades - 1) for i in range(n_shades)])

    means    = np.full((n_shades, NUM_SENSORS), np.nan)
    raw_rows = {sid: np.array(data[sid]) for sid in present_ids}

    for sid in present_ids:
        rows = raw_rows[sid]   # shape (samples, NUM_SENSORS)
        means[sid] = rows.mean(axis=0)

    slopes     = np.zeros(NUM_SENSORS)
    intercepts = np.zeros(NUM_SENSORS)
    r2         = np.zeros(NUM_SENSORS)

    # Power-law parameters  y = a - b * (L/100)^n   (per sensor)
    pow_a  = np.zeros(NUM_SENSORS)
    pow_b  = np.zeros(NUM_SENSORS)
    pow_n  = np.zeros(NUM_SENSORS)
    pow_r2 = np.zeros(NUM_SENSORS)

    # Fit only over shades we actually measured
    fit_mask = np.array([sid in data for sid in range(n_shades)])
    x_fit    = lightness_all[fit_mask]

    for k in range(NUM_SENSORS):
        y_fit = means[fit_mask, k]

        # ---- Linear fit (kept for diagnostics) -------------------------
        slope, intercept = np.polyfit(x_fit, y_fit, 1)
        slopes[k]     = slope
        intercepts[k] = intercept

        y_pred_lin = slope * x_fit + intercept
        ss_res_lin = float(np.sum((y_fit - y_pred_lin) ** 2))
        ss_tot     = float(np.sum((y_fit - y_fit.mean()) ** 2))
        r2[k]      = 1.0 - (ss_res_lin / ss_tot) if ss_tot > 0 else 0.0

        # ---- Power-law fit  y = a - b * (L/100)^n ----------------------
        #
        # a ≈ raw at L=0 (black end),   b ≈ black-white ADC range,
        # n ≈ 2 - 3 (printer-gamma + phototransistor roll-off).
        def _power(L, a, b, n):
            return a - b * (L / 100.0) ** n

        p0 = [float(y_fit[0]), max(float(y_fit[0] - y_fit[-1]), 1.0), 2.5]
        try:
            params, _ = curve_fit(_power, x_fit, y_fit, p0=p0, maxfev=20000)
            pow_a[k], pow_b[k], pow_n[k] = params
            y_pred_pow = _power(x_fit, *params)
            ss_res_pow = float(np.sum((y_fit - y_pred_pow) ** 2))
            pow_r2[k]  = 1.0 - (ss_res_pow / ss_tot) if ss_tot > 0 else 0.0
        except Exception as e:
            # Fall back to linear-equivalent power params so the firmware
            # still receives a valid (if bad) calibration.
            print(f"WARNING: sensor {k} power-law fit failed ({e}); "
                  f"falling back to linear-equivalent params.")
            pow_a[k] = float(y_fit[0])
            pow_b[k] = max(float(y_fit[0] - y_fit[-1]), 1.0)
            pow_n[k] = 1.0
            pow_r2[k] = r2[k]

    # Anchor points: shade 0 = black (must exist), shade n_shades-1 = white (must exist)
    if 0 not in data:
        print("ERROR: shade_id 0 (black) missing — cannot anchor CAL_BLACK.")
        sys.exit(1)
    if (n_shades - 1) not in data:
        print(f"ERROR: shade_id {n_shades-1} (white) missing — "
              f"cannot anchor CAL_WHITE.")
        sys.exit(1)

    black = means[0].copy()
    white = means[n_shades - 1].copy()
    ranges = black - white

    # Sanity check — range should be large (full black/white excursion)
    for k in range(NUM_SENSORS):
        if ranges[k] <= MIN_RANGE_WARN:
            print(f"WARNING: sensor {k} has a small black-white range "
                  f"({ranges[k]:.1f} ADC). Calibration may be unreliable.")

    return {
        "lightness":  lightness_all,
        "fit_mask":   fit_mask,
        "means":      means,
        "raw_rows":   raw_rows,
        "slopes":     slopes,
        "intercepts": intercepts,
        "r2":         r2,
        "pow_a":      pow_a,
        "pow_b":      pow_b,
        "pow_n":      pow_n,
        "pow_r2":     pow_r2,
        "white":      white,
        "black":      black,
        "ranges":     ranges,
        "n_shades":   n_shades,
    }


# -----------------------------------------------------------------------
# Calibration.h emitter
# -----------------------------------------------------------------------
def write_calibration_header(result, input_filename):
    """Overwrite 3Pi_CodeStub/Calibration.h with the fitted anchor points."""
    os.makedirs(os.path.dirname(CALIB_OUT_PATH), exist_ok=True)

    white  = result["white"]
    black  = result["black"]
    ranges = result["ranges"]
    pow_a  = result["pow_a"]
    pow_b  = result["pow_b"]
    pow_n  = result["pow_n"]
    pow_r2 = result["pow_r2"]
    lin_r2 = result["r2"]
    n      = result["n_shades"]

    def fmt(arr, prec=2):
        return ", ".join(f"{v:.{prec}f}" for v in arr)

    def fmt_r2(arr):
        return ", ".join(f"{v:.4f}" for v in arr)

    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    header = (
        "/*\n"
        " * Calibration.h — AUTO-GENERATED by transfer_function.py\n"
        " *\n"
        " * Per-sensor calibration for the 3Pi+ line sensors, consumed by\n"
        " * GradientCalibration.h in the GradientCentreFinder sketch.\n"
        " * Flashed into the robot as part of the sketch — no runtime\n"
        " * calibration is performed.\n"
        " *\n"
        f" * Source file : {os.path.basename(input_filename)}\n"
        f" * Generated   : {now}\n"
        f" * Shades used : {n}\n"
        " *\n"
        " * Transfer function (per sensor k) used by gradientToPercent():\n"
        " *\n"
        " *     pct = 100 * ( (CAL_A[k] - raw) / CAL_B[k] ) ^ (1 / CAL_N[k])\n"
        " *\n"
        " * clamped to [0, 100].  Convention: 0 % = black, 100 % = white.\n"
        " *\n"
        " * The inverse of the fitted forward model:\n"
        " *     raw = CAL_A[k] - CAL_B[k] * (L/100) ^ CAL_N[k]\n"
        " *\n"
        " * Justification: the phototransistor + pull-up divider plus the\n"
        " * printed-gamma reflectance of the test gradient produce a\n"
        " * power-law response, not a linear one.  Measured R^2 rises from\n"
        " * ~0.85 (linear) to ~0.995 (power-law).  See\n"
        " * TransferFunction_Analysis.md.\n"
        " *\n"
        " * The legacy CAL_WHITE / CAL_BLACK / CAL_RANGE arrays are kept\n"
        " * for diagnostics and for the retired linear path\n"
        " * (gradientToPercentLinear()).\n"
        " *\n"
        f" * Linear-fit R^2  per sensor : [{fmt_r2(lin_r2)}]\n"
        f" * Power-fit  R^2 per sensor : [{fmt_r2(pow_r2)}]\n"
        " *\n"
        " * DO NOT EDIT — regenerate by running transfer_function.py.\n"
        " */\n"
        "#ifndef _CALIBRATION_H\n"
        "#define _CALIBRATION_H\n"
        "\n"
        "// --- Power-law transfer function (ACTIVE) -----------------------\n"
        "// Forward model per sensor:  raw = CAL_A - CAL_B * (L/100)^CAL_N\n"
        "//   CAL_A ≈ raw ADC at L=0 % (black end, upper asymptote)\n"
        "//   CAL_B ≈ full black→white ADC excursion (scale)\n"
        "//   CAL_N ≈ curvature exponent (typ. 2–3; printer-gamma dominated)\n"
        f"const float CAL_A[5] = {{ {fmt(pow_a)} }};\n"
        f"const float CAL_B[5] = {{ {fmt(pow_b)} }};\n"
        f"const float CAL_N[5] = {{ {fmt(pow_n, prec=4)} }};\n"
        "\n"
        "// --- Linear anchors (legacy / diagnostics only) -----------------\n"
        "// Mean ADC reading on white surface (lightest patch, lowest ADC)\n"
        f"const float CAL_WHITE[5] = {{ {fmt(white)} }};\n"
        "\n"
        "// Mean ADC reading on black surface (darkest patch, highest ADC)\n"
        f"const float CAL_BLACK[5] = {{ {fmt(black)} }};\n"
        "\n"
        "// Range = CAL_BLACK - CAL_WHITE  (pre-computed)\n"
        f"const float CAL_RANGE[5] = {{ {fmt(ranges)} }};\n"
        "\n"
        "#endif\n"
    )

    with open(CALIB_OUT_PATH, "w", encoding="utf-8") as f:
        f.write(header)
    print(f"Wrote {CALIB_OUT_PATH}")


# -----------------------------------------------------------------------
# Diagnostic plot
# -----------------------------------------------------------------------
def save_plot(result, input_path):
    """5-panel diagnostic: raw scatter, per-shade means, linear fit per sensor."""
    lightness  = result["lightness"]
    fit_mask   = result["fit_mask"]
    means      = result["means"]
    raw_rows   = result["raw_rows"]
    slopes     = result["slopes"]
    intercepts = result["intercepts"]
    r2         = result["r2"]
    pow_a      = result["pow_a"]
    pow_b      = result["pow_b"]
    pow_n      = result["pow_n"]
    pow_r2     = result["pow_r2"]
    n_shades   = result["n_shades"]

    stem  = os.path.splitext(os.path.basename(input_path))[0]
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    fig, axes = plt.subplots(1, NUM_SENSORS, figsize=(4 * NUM_SENSORS, 4),
                             sharex=True, sharey=False)
    fig.suptitle(
        f"Sensor Transfer Functions — {n_shades} shades — {stamp}",
        fontsize=13,
    )

    x_fit = lightness[fit_mask]
    # Smooth line for fitted regression
    x_smooth = np.linspace(0, 100, 100)

    for k in range(NUM_SENSORS):
        ax = axes[k]

        # Raw samples as small dots
        for sid, rows in raw_rows.items():
            xs = np.full(rows.shape[0], lightness[sid])
            ax.scatter(xs, rows[:, k], s=8, alpha=0.3,
                       color="tab:blue", edgecolors="none")

        # Per-shade means (only where data exists)
        y_means = means[fit_mask, k]
        ax.plot(x_fit, y_means, marker="o", color="tab:orange",
                linewidth=1.2, label="Mean / shade")

        # Linear fit (legacy reference)
        y_line = slopes[k] * x_smooth + intercepts[k]
        ax.plot(x_smooth, y_line, "--", color="tab:green",
                linewidth=1.2, label=f"Linear (R²={r2[k]:.3f})")

        # Power-law fit (active transfer function)
        y_pow = pow_a[k] - pow_b[k] * (x_smooth / 100.0) ** pow_n[k]
        ax.plot(x_smooth, y_pow, "-", color="tab:red",
                linewidth=2.0, label=f"Power (R²={pow_r2[k]:.3f})")

        ax.set_title(f"Sensor {k} (DN{k+1})")
        ax.set_xlabel("Lightness (%)")
        if k == 0:
            ax.set_ylabel("Raw ADC (0–1023)")
        ax.set_xlim(-5, 105)
        ax.grid(True, alpha=0.3)

        # Annotation: power-law params + both R²s
        txt = (f"a = {pow_a[k]:.1f}\n"
               f"b = {pow_b[k]:.1f}\n"
               f"n = {pow_n[k]:.3f}\n"
               f"R²(pow) = {pow_r2[k]:.4f}\n"
               f"R²(lin) = {r2[k]:.4f}")
        ax.text(0.98, 0.98, txt, transform=ax.transAxes,
                ha="right", va="top", fontsize=8,
                bbox=dict(boxstyle="round,pad=0.3",
                          facecolor="white", alpha=0.85, edgecolor="grey"))

        if k == NUM_SENSORS - 1:
            ax.legend(loc="lower left", fontsize=8)

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    os.makedirs(DATA_DIR, exist_ok=True)
    out_path = os.path.join(DATA_DIR, f"{stem}_transfer_functions.png")
    plt.savefig(out_path, dpi=120)
    print(f"Saved plot: {out_path}")
    plt.show()


# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
def main():
    input_path = resolve_input_path(sys.argv)
    n_shades   = parse_shade_count(input_path)

    print(f"Input file : {input_path}")
    print(f"Shade count: {n_shades}")

    data   = load_data(input_path)
    result = fit_calibration(data, n_shades)

    # Report
    print("\nPer-sensor summary:")
    print(f"  {'k':>2}  {'white':>7}  {'black':>7}  {'range':>7}  "
          f"{'slope':>7}  {'linR²':>6}  {'a':>8}  {'b':>8}  "
          f"{'n':>6}  {'powR²':>6}")
    for k in range(NUM_SENSORS):
        print(f"  {k:>2}  "
              f"{result['white'][k]:>7.2f}  "
              f"{result['black'][k]:>7.2f}  "
              f"{result['ranges'][k]:>7.2f}  "
              f"{result['slopes'][k]:>7.3f}  "
              f"{result['r2'][k]:>6.3f}  "
              f"{result['pow_a'][k]:>8.2f}  "
              f"{result['pow_b'][k]:>8.2f}  "
              f"{result['pow_n'][k]:>6.3f}  "
              f"{result['pow_r2'][k]:>6.3f}")

    write_calibration_header(result, input_path)
    save_plot(result, input_path)


if __name__ == "__main__":
    main()
