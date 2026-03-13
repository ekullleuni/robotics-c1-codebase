"""
plot_gradient.py
----------------
Plots Phase 2 (gradient test) data from a LineSensorTest CSV file.

Usage:
    python plot_gradient.py data/20260311_141211.csv

If no file is given, uses the most recent file in data/.

Expected input rows (9 fields):
    gradient_id, timestamp_ms, dist_left_mm, dist_right_mm, s0, s1, s2, s3, s4

Distance is computed on the robot from encoder counts using countsToMM():
    dist_mm = counts / COUNTS_PER_WHEEL_REV * WHEEL_CIRCUMFERENCE_MM
    (COUNTS_PER_WHEEL_REV=358.3, WHEEL_CIRCUMFERENCE_MM=100.53)

The x-axis is the mean of dist_left_mm and dist_right_mm, representing
the robot's forward travel distance from the start of each gradient strip.
"""

import sys
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# -----------------------------------------------------------------------
# Sensor labels and colours
# -----------------------------------------------------------------------
SENSOR_NAMES  = ["DN1 (left)", "DN2", "DN3 (centre)", "DN4", "DN5 (right)"]
SENSOR_COLORS = ["#e63946", "#457b9d", "#2a9d8f", "#e9c46a", "#f4a261"]

NUM_SENSORS = 5


# -----------------------------------------------------------------------
# Data loading
# -----------------------------------------------------------------------
def load_gradient_data(filepath):
    """
    Parse a LineSensorTest CSV file and return gradient data.

    Returns
    -------
    strips : dict  {gradient_id (int): np.ndarray shape (N, 7)}
                   columns: [dist_left_mm, dist_right_mm, s0, s1, s2, s3, s4]
    garbled : int  number of lines that could not be parsed
    """
    strips  = {}
    garbled = 0
    gradient_rows_found = 0

    with open(filepath, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split(",")
            if len(parts) != 9:
                # Shade rows have 8 fields; thin-line rows have 10 fields
                garbled += 1
                continue

            try:
                gradient_id   = int(parts[0])
                # parts[1] = timestamp_ms — kept in data but not used for plot x-axis
                dist_left_mm  = float(parts[2])
                dist_right_mm = float(parts[3])
                sensors       = [int(parts[4 + i]) for i in range(NUM_SENSORS)]
            except (ValueError, IndexError):
                garbled += 1
                continue

            row = [dist_left_mm, dist_right_mm] + sensors
            if gradient_id not in strips:
                strips[gradient_id] = []
            strips[gradient_id].append(row)
            gradient_rows_found += 1

    # Convert lists to numpy arrays
    for gid in strips:
        strips[gid] = np.array(strips[gid])   # shape (N, 7)

    return strips, garbled, gradient_rows_found


# -----------------------------------------------------------------------
# Plot
# -----------------------------------------------------------------------
def plot_gradient(strips, filepath):
    """
    One figure per gradient strip, 5 subplots per figure (one per sensor).
    X axis: distance from strip start (mm)  — mean of left/right wheel distances
    Y axis: raw ADC reading (autoscaled)
    """
    strip_ids = sorted(strips.keys())

    for gid in strip_ids:
        data = strips[gid]          # shape (N, 7)
        dist_mm = (data[:, 0] + data[:, 1]) / 2.0  # mean wheel distance = forward travel

        fig, axes = plt.subplots(
            NUM_SENSORS, 1,
            figsize=(14, 3.0 * NUM_SENSORS),
            sharex=True
        )
        fig.suptitle(
            f"Line Sensor Response vs Distance — Gradient Strip {gid}\n"
            f"(Phase 2 — Gradient Test,  {len(dist_mm)} samples)",
            fontsize=13, fontweight="bold", y=1.01
        )

        for sensor_idx, ax in enumerate(axes):
            color = SENSOR_COLORS[sensor_idx]
            label = SENSOR_NAMES[sensor_idx]
            readings = data[:, 2 + sensor_idx]  # columns 2-6 are s0-s4

            ax.plot(dist_mm, readings, color=color, linewidth=1.0, alpha=0.8)

            # Overlay a smoothed trend (rolling mean, window ~20 samples)
            window = min(20, max(3, len(readings) // 20))
            if len(readings) >= window:
                kernel = np.ones(window) / window
                smoothed = np.convolve(readings, kernel, mode="valid")
                # Align smoothed to centre of window
                offset = window // 2
                ax.plot(dist_mm[offset: offset + len(smoothed)],
                        smoothed,
                        color=color, linewidth=2.0, linestyle="--",
                        alpha=1.0, label=f"rolling mean (n={window})")
                ax.legend(loc="upper right", fontsize=7)

            ax.set_ylabel("ADC reading", fontsize=9)
            ax.set_title(label, fontsize=10, color=color, loc="left")
            ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
            ax.grid(axis="y", linestyle="--", alpha=0.4)
            ax.grid(axis="x", linestyle=":", alpha=0.3)

        axes[-1].set_xlabel(
            "Distance from strip start (mm)  [mean of left & right wheel encoders]",
            fontsize=10
        )
        axes[-1].xaxis.set_minor_locator(ticker.AutoMinorLocator())

        plt.tight_layout()

        # Save alongside the CSV
        base = filepath.replace(".csv", "")
        out_path = f"{base}_gradient{gid}_plot.png"
        fig.savefig(out_path, dpi=150, bbox_inches="tight")
        print(f"  Plot saved: {out_path}")

    plt.show()


# -----------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------
def main():
    # Pick file
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        files = sorted(glob.glob("data/*.csv"))
        if not files:
            print("No CSV files found in data/. Run log_sensors.py first.")
            sys.exit(1)
        filepath = files[-1]
        print(f"Using most recent file: {filepath}")

    if not os.path.exists(filepath):
        print(f"File not found: {filepath}")
        sys.exit(1)

    print(f"Loading {filepath} ...")
    strips, garbled, gradient_rows = load_gradient_data(filepath)

    if gradient_rows == 0:
        print()
        print("ERROR: No gradient test data (9-field rows) found in this file.")
        print("The file appears to contain shade or thin-line data only.")
        print()
        print("To collect gradient data:")
        print("  1. Flash LineSensorTest.ino to the robot")
        print("  2. Run:  python3 log_sensors.py")
        print("  3. Press BUTTON A twice to skip to Phase 2 (Gradient Test)")
        print("  4. Place robot at black end of gradient strip and press BUTTON B")
        print("  5. Push robot slowly along the strip; press B again to stop")
        sys.exit(1)

    print(f"  Gradient rows loaded : {gradient_rows}")
    print(f"  Gradient strips      : {len(strips)}")
    print(f"  Garbled lines        : {garbled}")

    for gid in sorted(strips.keys()):
        data = strips[gid]
        dist_mm = (data[:, 0] + data[:, 1]) / 2.0
        print(f"  Strip {gid}: {len(dist_mm):4d} samples, "
              f"distance {dist_mm.min():.1f} – {dist_mm.max():.1f} mm")

    plot_gradient(strips, filepath)


if __name__ == "__main__":
    main()
