"""
plot_shade.py
-------------
Plots Phase 1 (shade test) data from a LineSensorTest CSV file.

Usage:
    python plot_shade.py data/20260311_141211.csv

If no file is given, uses the most recent file in data/.

Expected input rows (8 fields):
    shade_id, sample, timestamp_ms, s0, s1, s2, s3, s4

The 25 shades are evenly spaced in HSL lightness from 0 % (black,
shade_id=0) to 100 % (white, shade_id=24).
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

NUM_SHADES  = 25
NUM_SENSORS = 5

# HSL lightness values for the 25 patches (0 % = black, 100 % = white)
# Evenly spaced: 0, 4.17, 8.33, ..., 100  (step = 100/24)
LIGHTNESS = [i * 100 / (NUM_SHADES - 1) for i in range(NUM_SHADES)]


# -----------------------------------------------------------------------
# Data loading
# -----------------------------------------------------------------------
def load_shade_data(filepath):
    """
    Parse a LineSensorTest CSV file and return shade data.

    Returns
    -------
    data : dict  {shade_id (int): np.ndarray of shape (N_samples, 5)}
    garbled : int   number of lines that could not be parsed
    """
    data    = {i: [] for i in range(NUM_SHADES)}
    garbled = 0
    shade_rows_found = 0

    with open(filepath, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split(",")
            if len(parts) != 8:
                # Not a shade row (gradient rows have 9 fields)
                garbled += 1
                continue

            try:
                shade_id  = int(parts[0])
                # parts[1] = sample index, parts[2] = timestamp — not needed
                sensors   = [int(parts[3 + i]) for i in range(NUM_SENSORS)]
            except (ValueError, IndexError):
                garbled += 1
                continue

            if 0 <= shade_id < NUM_SHADES:
                data[shade_id].append(sensors)
                shade_rows_found += 1

    # Convert lists to numpy arrays
    for sid in data:
        data[sid] = np.array(data[sid]) if data[sid] else np.empty((0, NUM_SENSORS))

    return data, garbled, shade_rows_found


# -----------------------------------------------------------------------
# Plot
# -----------------------------------------------------------------------
def plot_shade(data):
    """
    5 subplots, one per sensor.
    X axis: HSL lightness (%)
    Y axis: raw ADC reading (0–1023)
    Box plots at each shade position.
    """
    fig, axes = plt.subplots(
        NUM_SENSORS, 1,
        figsize=(14, 3.2 * NUM_SENSORS),
        sharex=True
    )
    fig.suptitle("Line Sensor Response vs Surface Lightness\n(Phase 1 — 25 Shades of Gray)",
                 fontsize=14, fontweight="bold", y=1.01)

    positions = LIGHTNESS  # x positions for box plots

    for sensor_idx, ax in enumerate(axes):
        color  = SENSOR_COLORS[sensor_idx]
        label  = SENSOR_NAMES[sensor_idx]

        # Build list of arrays: one per shade, containing N readings for this sensor
        readings_per_shade = []
        n_samples_list     = []
        for sid in range(NUM_SHADES):
            arr = data[sid]
            if arr.shape[0] > 0:
                readings_per_shade.append(arr[:, sensor_idx].tolist())
                n_samples_list.append(arr.shape[0])
            else:
                readings_per_shade.append([])
                n_samples_list.append(0)

        # --- Box plots ---
        # Only plot shades that have data
        valid_pos   = [positions[i] for i in range(NUM_SHADES) if n_samples_list[i] > 0]
        valid_data  = [readings_per_shade[i] for i in range(NUM_SHADES) if n_samples_list[i] > 0]

        if valid_data:
            bp = ax.boxplot(
                valid_data,
                positions=valid_pos,
                widths=2.8,            # box width in % lightness units
                patch_artist=True,
                medianprops=dict(color="black", linewidth=1.5),
                whiskerprops=dict(color=color, linewidth=1),
                capprops=dict(color=color, linewidth=1),
                flierprops=dict(marker="o", markersize=2,
                                markerfacecolor=color, alpha=0.4,
                                linestyle="none"),
                boxprops=dict(facecolor=color, alpha=0.5, edgecolor=color)
            )

        # --- Overlay means as connected line ---
        means = [np.mean(readings_per_shade[i])
                 if n_samples_list[i] > 0 else np.nan
                 for i in range(NUM_SHADES)]
        ax.plot(positions, means, "o-", color=color, linewidth=1.5,
                markersize=4, zorder=5, label="mean")

        # --- Formatting ---
        ax.set_ylabel("ADC reading", fontsize=9)
        ax.set_title(label, fontsize=10, color=color, loc="left")
        ax.yaxis.set_minor_locator(ticker.MultipleLocator(50))
        ax.grid(axis="y", linestyle="--", alpha=0.4)
        ax.grid(axis="x", linestyle=":", alpha=0.3)

        # Annotate missing shades
        missing = [i for i in range(NUM_SHADES) if n_samples_list[i] == 0]
        if missing:
            ax.text(0.99, 0.05,
                    f"No data for shade(s): {missing}",
                    transform=ax.transAxes, ha="right", va="bottom",
                    fontsize=7, color="grey")

    # Shared x axis
    axes[-1].set_xlabel("Surface lightness — HSL L% (0 = black, 100 = white)", fontsize=10)
    axes[-1].set_xlim(-3, 103)
    axes[-1].xaxis.set_major_locator(ticker.MultipleLocator(10))
    axes[-1].xaxis.set_minor_locator(ticker.MultipleLocator(100 / (NUM_SHADES - 1)))

    # X tick labels at every shade position
    axes[-1].set_xticks(positions)
    axes[-1].set_xticklabels([f"{l:.0f}" for l in positions], fontsize=7, rotation=45)

    plt.tight_layout()
    return fig


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
    data, garbled, shade_rows = load_shade_data(filepath)

    if shade_rows == 0:
        print()
        print("ERROR: No shade test data (8-field rows) found in this file.")
        print("The file appears to contain gradient or thin-line data only.")
        print()
        print("To collect shade data:")
        print("  1. Flash LineSensorTest.ino to the robot")
        print("  2. Run:  python3 log_sensors.py")
        print("  3. Press BUTTON A to enter Phase 1 (Shade Test)")
        print("  4. Place robot on each shade patch and press BUTTON B")
        print("  5. Repeat for all 25 patches")
        sys.exit(1)

    total_readings = sum(d.shape[0] for d in data.values())
    shades_with_data = sum(1 for d in data.values() if d.shape[0] > 0)
    print(f"  Shade rows loaded : {shade_rows}")
    print(f"  Shades with data  : {shades_with_data} / {NUM_SHADES}")
    print(f"  Garbled lines     : {garbled}")
    for sid in range(NUM_SHADES):
        n = data[sid].shape[0]
        pct = LIGHTNESS[sid]
        if n > 0:
            means = data[sid].mean(axis=0)
            print(f"  Shade {sid:2d} (L={pct:5.1f}%): {n:3d} samples — "
                  f"means: {', '.join(f'{m:.0f}' for m in means)}")
        else:
            print(f"  Shade {sid:2d} (L={pct:5.1f}%): NO DATA")

    fig = plot_shade(data)

    # Save alongside the data file
    out_path = filepath.replace(".csv", "_shade_plot.png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"\nPlot saved to: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
