"""
Line Sensor Resolution Analysis
================================
Analyses the usable resolution of reflectance line sensors on a 3pi+ robot
from one or more 25-shade grayscale calibration datasets.

Multiple files are combined per shade — min/max spans ALL readings across
all sessions, capturing both electronic noise and between-placement variation.

The 25 shade patches are ground truth: shade 0 is darkest, shade 24 is
lightest.  Sensors read high for dark, low for light (expect decreasing).

Worst-case error is measured directly: for consecutive shades i and i+1,
we compute min(readings_i) - max(readings_i+1).  If positive, the two
shades are always distinguishable.  If negative, they overlap.

Data format: shade_id, sample, timestamp_ms, s0, s1, s2, s3, s4
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

SENSORS = ["s0", "s1", "s2", "s3", "s4"]
SETTLE_SAMPLES = 7  # discard initial settling transient per capture session


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------


def load_shade_data(filepath: str) -> dict[int, dict[str, np.ndarray]]:
    """Parse a single shade-test CSV and return per-shade, per-sensor arrays."""
    raw: dict[int, dict[str, list[int]]] = {}

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


def combine_datasets(filepaths: list[str]) -> dict[int, dict[str, np.ndarray]]:
    """Load multiple files and concatenate stable readings per shade.

    Each file's first SETTLE_SAMPLES per shade are discarded independently
    (each placement has its own settling transient), then the remaining
    stable readings are concatenated.
    """
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


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------


def compute_shade_stats(data: dict):
    """Per-shade mean, std, min, max from combined readings."""
    means = {s: [] for s in SENSORS}
    stds = {s: [] for s in SENSORS}
    mins = {s: [] for s in SENSORS}
    maxs = {s: [] for s in SENSORS}
    counts = []

    for shade_id in sorted(data.keys()):
        counts.append(len(data[shade_id][SENSORS[0]]))
        for s in SENSORS:
            arr = data[shade_id][s]
            means[s].append(np.mean(arr))
            stds[s].append(np.std(arr, ddof=1))
            mins[s].append(int(np.min(arr)))
            maxs[s].append(int(np.max(arr)))

    for s in SENSORS:
        means[s] = np.array(means[s])
        stds[s] = np.array(stds[s])
        mins[s] = np.array(mins[s])
        maxs[s] = np.array(maxs[s])
    return means, stds, mins, maxs, counts


def compute_worst_case_gaps(mins: dict, maxs: dict):
    """For each consecutive pair, compute min[i] - max[i+1]."""
    results = {s: [] for s in SENSORS}
    for s in SENSORS:
        n = len(mins[s])
        for i in range(n - 1):
            gap = int(mins[s][i]) - int(maxs[s][i + 1])
            results[s].append(
                {
                    "pair": (i, i + 1),
                    "gap": gap,
                    "min_darker": int(mins[s][i]),
                    "max_lighter": int(maxs[s][i + 1]),
                }
            )
    return results


def compute_all_pair_gaps(mins: dict, maxs: dict):
    """For every pair (i, j) with i < j, check min[i] - max[j]."""
    n = len(mins[SENSORS[0]])
    results = {s: {} for s in SENSORS}
    for s in SENSORS:
        for gap_size in range(1, n):
            n_pairs = n - gap_size
            overlaps = 0
            worst_overlap = 0.0
            for i in range(n_pairs):
                j = i + gap_size
                wc_gap = int(mins[s][i]) - int(maxs[s][j])
                if wc_gap < 0:
                    overlaps += 1
                    worst_overlap = max(worst_overlap, abs(wc_gap))
            results[s][gap_size] = {
                "n_pairs": n_pairs,
                "overlaps": overlaps,
                "worst_overlap": worst_overlap,
                "clear_pct": 100 * (1 - overlaps / n_pairs),
            }
    return results


def compute_resolution(means, mins, maxs, wc_gaps, all_pair_gaps):
    """Compute resolution metrics."""
    metrics = {}
    for s in SENSORS:
        total_range = means[s].max() - means[s].min()

        overlaps = [g for g in wc_gaps[s] if g["gap"] < 0]
        worst_overlap = max((abs(g["gap"]) for g in overlaps), default=0.0)
        n_overlapping = len(overlaps)

        # Min gap for 100% clear separation
        min_gap_100 = None
        n = len(means[s])
        for gap_size in range(1, n):
            if all_pair_gaps[s][gap_size]["overlaps"] == 0:
                min_gap_100 = gap_size
                break

        # Greedy class allocation: walk forward, place new class when
        # guaranteed separation from current anchor
        classes = [0]
        for j in range(1, n):
            anchor = classes[-1]
            if int(mins[s][anchor]) - int(maxs[s][j]) > 0:
                classes.append(j)
        n_classes = len(classes)

        metrics[s] = {
            "total_range": total_range,
            "worst_overlap": worst_overlap,
            "n_overlapping": n_overlapping,
            "min_gap_100": min_gap_100,
            "n_classes": n_classes,
            "class_shades": classes,
            "bits": np.log2(n_classes) if n_classes > 1 else 0,
        }
    return metrics


def compute_drift(data, shade_ids, sensor="s2"):
    """Check for temporal drift within a shade capture."""
    results = []
    for shade_id in shade_ids:
        arr = data[shade_id][sensor]
        first_10 = arr[:10]
        last_10 = arr[-10:]
        drift = np.mean(last_10) - np.mean(first_10)
        results.append(
            {
                "shade": shade_id,
                "sensor": sensor,
                "first_10_mean": np.mean(first_10),
                "last_10_mean": np.mean(last_10),
                "drift": drift,
            }
        )
    return results


# ---------------------------------------------------------------------------
# Printing
# ---------------------------------------------------------------------------


def print_report(
    means,
    stds,
    mins,
    maxs,
    wc_gaps,
    all_pair_gaps,
    metrics,
    drift_results,
    n_shades,
    n_files,
    counts,
):
    sep = "=" * 80

    print(sep)
    print("LINE SENSOR RESOLUTION ANALYSIS")
    print(sep)
    samples_str = (
        f"{counts[0]}" if len(set(counts)) == 1 else f"{min(counts)}-{max(counts)}"
    )
    print(
        f"\nShades: {n_shades}  |  Files combined: {n_files}"
        f"  |  Stable samples per shade: {samples_str}"
    )
    print(f"Ground truth: shade 0 = darkest, shade {n_shades - 1} = lightest.")
    print("Sensors read high for dark, low for light (expect decreasing).")
    print("\nError method: worst-case overlap = min(shade_i) - max(shade_i+1)")
    print("  min/max spans ALL readings across all sessions.")

    # Per-shade stats
    print(f"\n{sep}")
    print("PER-SHADE STATISTICS")
    print(sep)
    print(f"{'Shade':>5} {'N':>4} |", end="")
    for s in SENSORS:
        print(
            f"  {s + '_mean':>7} {s + '_min':>6} {s + '_max':>6} {s + '_rng':>5} |",
            end="",
        )
    print()
    print("-" * 130)
    for i in range(n_shades):
        print(f"{i:>5} {counts[i]:>4} |", end="")
        for s in SENSORS:
            rng = int(maxs[s][i]) - int(mins[s][i])
            print(
                f"  {means[s][i]:>7.1f} {mins[s][i]:>6} {maxs[s][i]:>6} {rng:>5} |",
                end="",
            )
        print()

    # Worst-case gaps
    print(f"\n{sep}")
    print("CONSECUTIVE WORST-CASE GAPS: min[i] - max[i+1]")
    print(sep)
    print(f"  {'Pair':>6} |", end="")
    for s in SENSORS:
        print(f"  {s:>6} |", end="")
    print()
    print(f"  {'-' * 6}-+", end="")
    for _ in SENSORS:
        print(f"-{'-' * 7}-+", end="")
    print()
    for idx in range(n_shades - 1):
        pair_str = f"{idx}-{idx + 1}"
        print(f"  {pair_str:>6} |", end="")
        for s in SENSORS:
            g = wc_gaps[s][idx]["gap"]
            marker = " *" if g < 0 else "  "
            print(f"  {g:>+5}{marker}|", end="")
        print()
    print("  (* = overlap: shades NOT always distinguishable)")

    # Overlap summary
    print(f"\n{sep}")
    print("OVERLAP SUMMARY")
    print(sep)
    for s in SENSORS:
        m = metrics[s]
        overlaps = [g for g in wc_gaps[s] if g["gap"] < 0]
        if overlaps:
            pairs_str = ", ".join(
                f"{g['pair'][0]}->{g['pair'][1]} ({g['gap']:+})" for g in overlaps
            )
            print(
                f"  {s}: {m['n_overlapping']} overlapping pair(s): {pairs_str}"
                f"  |  worst: {m['worst_overlap']:.0f} counts"
            )
        else:
            print(f"  {s}: no overlaps (all consecutive shades always separable)")

    # Gap analysis
    print(f"\n{sep}")
    print("MINIMUM SHADE GAP FOR GUARANTEED SEPARATION")
    print(sep)
    for s in SENSORS:
        gap = metrics[s]["min_gap_100"]
        if gap is not None:
            print(f"  {s}: gap >= {gap} shades -> 100% worst-case separation")
        else:
            print(f"  {s}: never achieves guaranteed separation!")

    print(f"\n  Guaranteed separation % by gap (s2):")
    print(f"    {'Gap':>4} | {'Pairs':>5} | {'Overlaps':>8} | {'Clear':>8}")
    for gap_size in range(1, min(8, n_shades)):
        info = all_pair_gaps["s2"][gap_size]
        print(
            f"    {gap_size:>4} | {info['n_pairs']:>5} | {info['overlaps']:>8} | "
            f"{info['clear_pct']:>7.1f}%"
        )

    # Distinguishable classes
    print(f"\n{sep}")
    print("GUARANTEED DISTINGUISHABLE CLASSES (greedy allocation)")
    print(sep)
    for s in SENSORS:
        m = metrics[s]
        print(
            f"  {s}: {m['n_classes']} classes at shades {m['class_shades']}"
            f"  ({m['bits']:.1f} bits)"
        )

    # Drift
    print(f"\n{sep}")
    print("DRIFT ANALYSIS")
    print(sep)
    for d in drift_results:
        print(
            f"  Shade {d['shade']:>2}, {d['sensor']}: "
            f"first 10 mean={d['first_10_mean']:.2f}, "
            f"last 10 mean={d['last_10_mean']:.2f}, "
            f"drift={d['drift']:+.2f}"
        )


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


def plot_analysis(
    means, stds, mins, maxs, wc_gaps, all_pair_gaps, metrics, n_shades, n_files
):
    shade_idx = np.arange(n_shades)
    fig = plt.figure(figsize=(14, 18))
    gs = gridspec.GridSpec(
        4, 2, hspace=0.38, wspace=0.30, left=0.08, right=0.95, top=0.96, bottom=0.04
    )

    suptitle = "Line Sensor Resolution Analysis"
    if n_files > 1:
        suptitle += f" ({n_files} sessions combined)"
    fig.suptitle(suptitle, fontsize=14, fontweight="bold", y=0.99)

    # ------------------------------------------------------------------
    # Panel 1: Shade means with min/max bands (full width)
    # ------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, :])
    for s in SENSORS:
        ax1.plot(
            shade_idx,
            means[s],
            "o-",
            color=SENSOR_COLOURS[s],
            markersize=4,
            linewidth=1.5,
            label=s,
            alpha=0.8,
        )
        ax1.fill_between(
            shade_idx, mins[s], maxs[s], color=SENSOR_COLOURS[s], alpha=0.12
        )
    ax1.set_xlabel("Shade index (0 = black, 24 = white)")
    ax1.set_ylabel("ADC counts")
    ax1.set_title(
        "Shade means with min/max range across all sessions", fontweight="bold"
    )
    ax1.legend(loc="upper right", framealpha=0.9)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(-0.5, n_shades - 0.5)

    # ------------------------------------------------------------------
    # Panel 2: Worst-case gap for s2 (full width)
    # ------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[1, :])
    gaps_s2 = [g["gap"] for g in wc_gaps["s2"]]
    pair_labels = [f"{i}-{i + 1}" for i in range(n_shades - 1)]
    colours = ["#E24B4A" if g < 0 else "#1D9E75" for g in gaps_s2]
    ax2.bar(range(len(gaps_s2)), gaps_s2, color=colours, alpha=0.8)
    ax2.axhline(0, color="black", linewidth=1, alpha=0.5)
    ax2.set_xticks(range(len(pair_labels)))
    ax2.set_xticklabels(pair_labels, rotation=45, ha="right", fontsize=7)
    ax2.set_xlabel("Adjacent shade pair")
    ax2.set_ylabel("min[i] − max[i+1] (counts)")
    ax2.set_title(
        "Worst-case gap for s2 (green = always separable, red = overlap)",
        fontweight="bold",
    )
    ax2.grid(True, axis="y", alpha=0.3)

    # ------------------------------------------------------------------
    # Panel 3: Per-shade spread (max - min) (mid-left)
    # ------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[2, 0])
    bar_width = 0.15
    offsets = np.arange(len(SENSORS)) - (len(SENSORS) - 1) / 2
    for i, s in enumerate(SENSORS):
        spreads = maxs[s] - mins[s]
        ax3.bar(
            shade_idx + offsets[i] * bar_width,
            spreads,
            width=bar_width,
            color=SENSOR_COLOURS[s],
            alpha=0.75,
            label=s,
        )
    ax3.set_xlabel("Shade index")
    ax3.set_ylabel("max − min (counts)")
    ax3.set_title("Per-shade reading spread (all sessions)", fontweight="bold")
    ax3.legend(loc="upper left", fontsize=8, framealpha=0.9)
    ax3.grid(True, axis="y", alpha=0.3)
    ax3.set_xlim(-0.5, n_shades - 0.5)

    # ------------------------------------------------------------------
    # Panel 4: Worst overlap per sensor (mid-right)
    # ------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[2, 1])
    x_pos = np.arange(len(SENSORS))
    overlaps = [metrics[s]["worst_overlap"] for s in SENSORS]
    n_overlapping = [metrics[s]["n_overlapping"] for s in SENSORS]
    ax4.bar(x_pos, overlaps, color="#E24B4A", alpha=0.8)
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels(SENSORS)
    ax4.set_ylabel("Worst overlap (counts)")
    ax4.set_title("Worst-case overlap per sensor", fontweight="bold")
    ax4.grid(True, axis="y", alpha=0.3)
    for i, (o, n) in enumerate(zip(overlaps, n_overlapping)):
        ax4.text(
            i, o + 0.2, f"{o:.0f} ({n} pairs)", ha="center", va="bottom", fontsize=9
        )

    # ------------------------------------------------------------------
    # Panel 5: Guaranteed distinguishable classes (bottom-left)
    # ------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[3, 0])
    n_classes = [metrics[s]["n_classes"] for s in SENSORS]
    bits = [metrics[s]["bits"] for s in SENSORS]
    ax5.bar(x_pos, n_classes, color=[SENSOR_COLOURS[s] for s in SENSORS], alpha=0.8)
    ax5.set_xticks(x_pos)
    ax5.set_xticklabels(SENSORS)
    ax5.set_ylabel("Guaranteed distinct classes")
    ax5.set_title("Distinguishable grey levels (worst-case)", fontweight="bold")
    ax5.grid(True, axis="y", alpha=0.3)
    for i, (nc, b) in enumerate(zip(n_classes, bits)):
        ax5.text(
            i, nc + 0.2, f"{nc} ({b:.1f} bits)", ha="center", va="bottom", fontsize=9
        )

    # ------------------------------------------------------------------
    # Panel 6: Guaranteed separation % by gap (bottom-right)
    # ------------------------------------------------------------------
    ax6 = fig.add_subplot(gs[3, 1])
    max_gap_show = min(12, n_shades - 1)
    for s in SENSORS:
        gaps = range(1, max_gap_show + 1)
        pcts = [all_pair_gaps[s][g]["clear_pct"] for g in gaps]
        ax6.plot(
            list(gaps),
            pcts,
            "o-",
            color=SENSOR_COLOURS[s],
            markersize=4,
            linewidth=1.5,
            label=s,
            alpha=0.8,
        )
    ax6.axhline(100, color="black", linewidth=0.5, linestyle="--", alpha=0.3)
    ax6.set_xlabel("Shade gap (distance between compared shades)")
    ax6.set_ylabel("Guaranteed separable (%)")
    ax6.set_title("Worst-case separation vs shade gap", fontweight="bold")
    ax6.legend(loc="lower right", fontsize=8, framealpha=0.9)
    ax6.grid(True, alpha=0.3)
    ax6.set_xlim(0.5, max_gap_show + 0.5)

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

    n_files = len(filepaths)
    print(f"Loading {n_files} file(s) ...")
    for fp in filepaths:
        print(f"  - {fp}")

    data = combine_datasets([str(fp) for fp in filepaths])
    n_shades = len(data)

    means, stds, mins, maxs, counts = compute_shade_stats(data)
    wc_gaps = compute_worst_case_gaps(mins, maxs)
    all_pair_gaps = compute_all_pair_gaps(mins, maxs)
    metrics = compute_resolution(means, mins, maxs, wc_gaps, all_pair_gaps)
    drift_results = compute_drift(data, [0, n_shades // 2, n_shades - 1])

    print_report(
        means,
        stds,
        mins,
        maxs,
        wc_gaps,
        all_pair_gaps,
        metrics,
        drift_results,
        n_shades,
        n_files,
        counts,
    )

    print("\nGenerating plots ...")
    fig = plot_analysis(
        means, stds, mins, maxs, wc_gaps, all_pair_gaps, metrics, n_shades, n_files
    )

    out_path = filepaths[0].with_name(filepaths[0].stem + "_analysis.png")
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    print(f"Figure saved to {out_path}")

    plt.show()


if __name__ == "__main__":
    main()
