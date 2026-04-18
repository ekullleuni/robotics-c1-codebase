"""
regression_analysis.py
----------------------
Companion to transfer_function.py.  Where that script assumes a linear
fit, this one numerically determines which regression family best
matches the measured raw-ADC vs. lightness curve produced by
log_sensors.py.

Models tested per sensor:
    linear      :  y = a*x + b
    quadratic   :  y = a*x^2 + b*x + c
    cubic       :  y = a*x^3 + b*x^2 + c*x + d
    exponential :  y = a*exp(-b*x) + c           (decaying)
    logarithmic :  y = a*ln(x + 1) + b
    power       :  y = a - b*(x/100)^n           (reflectance-style)
    sigmoid     :  y = d + (a - d)/(1 + (x/c)^b)  (4-PL logistic)

For every model/sensor the script reports R^2, adjusted R^2, RMSE
and AIC, and produces an overlay plot so the non-linearity is
visually obvious.

Usage:
    python regression_analysis.py                       # newest Nshades_*.csv
    python regression_analysis.py data/26shades_*.csv   # explicit file
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

DATA_DIR    = "data"
NUM_SENSORS = 5


# -----------------------------------------------------------------------
# Data IO (mirrors transfer_function.py so results are comparable)
# -----------------------------------------------------------------------
def resolve_input_path(argv):
    if len(argv) > 1:
        if not os.path.exists(argv[1]):
            sys.exit(f"ERROR: file not found: {argv[1]}")
        return argv[1]
    candidates = sorted(
        glob.glob(os.path.join(DATA_DIR, "*shades_*.csv")),
        key=os.path.getmtime,
    )
    if not candidates:
        sys.exit(f"ERROR: no {DATA_DIR}/Nshades_*.csv files found.")
    return candidates[-1]


def parse_shade_count(filepath):
    m = re.search(r"(\d+)shades_", os.path.basename(filepath))
    if not m:
        sys.exit(f"ERROR: bad filename: {filepath}")
    return int(m.group(1))


def load_data(filepath):
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
# Candidate models
# -----------------------------------------------------------------------
def f_linear(x, a, b):
    return a * x + b

def f_quad(x, a, b, c):
    return a * x * x + b * x + c

def f_cubic(x, a, b, c, d):
    return a * x**3 + b * x**2 + c * x + d

def f_exp(x, a, b, c):
    # Decaying exponential: raw drops as lightness rises
    return a * np.exp(-b * x) + c

def f_log(x, a, b):
    # +1 avoids ln(0)
    return a * np.log(x + 1.0) + b

def f_power(x, a, b, n):
    # Reflectance-gamma style: raw = base - k * (L/100)^gamma
    # Keeps curve positive-valued; mirrors sRGB gamma physics.
    return a - b * (x / 100.0) ** n

def f_sigmoid(x, a, b, c, d):
    # 4-parameter logistic (Hill).  a = upper asymptote, d = lower
    # asymptote, c = inflection point in x, b = slope.
    # Guard against x==0 in the power term.
    xs = np.where(x <= 0, 1e-6, x)
    return d + (a - d) / (1.0 + (xs / c) ** b)


MODELS = [
    # name, function, initial-guess-fn(x,y), param-count, nice_label
    ("linear",      f_linear,
        lambda x, y: [ (y[-1]-y[0])/(x[-1]-x[0]+1e-9), y[0] ],
        2, "Linear (a·x + b)"),
    ("quadratic",   f_quad,
        lambda x, y: [0.0, (y[-1]-y[0])/(x[-1]-x[0]+1e-9), y[0]],
        3, "Quadratic (a·x² + b·x + c)"),
    ("cubic",       f_cubic,
        lambda x, y: [0.0, 0.0, (y[-1]-y[0])/(x[-1]-x[0]+1e-9), y[0]],
        4, "Cubic (a·x³ + b·x² + c·x + d)"),
    ("exponential", f_exp,
        lambda x, y: [max(y[0]-y[-1], 1.0), 0.02, y[-1]],
        3, "Exponential (a·e^(-b·x) + c)"),
    ("logarithmic", f_log,
        lambda x, y: [-(y[-1]-y[0])/np.log((x[-1]+1)/(x[0]+1)+1e-9), y[0]],
        2, "Logarithmic (a·ln(x+1) + b)"),
    ("power",       f_power,
        lambda x, y: [y[0], max(y[0]-y[-1], 1.0), 1.5],
        3, "Power (a − b·(x/100)^n)"),
    ("sigmoid",     f_sigmoid,
        lambda x, y: [y[0], 2.0, 50.0, y[-1]],
        4, "Sigmoid (4-PL logistic)"),
]


# -----------------------------------------------------------------------
# Fit + score a single model
# -----------------------------------------------------------------------
def fit_one(model_fn, p0, x, y):
    """Return (params, y_pred) or (None, None) if the fit fails."""
    try:
        params, _ = curve_fit(model_fn, x, y, p0=p0, maxfev=20000)
        y_pred = model_fn(x, *params)
        if not np.all(np.isfinite(y_pred)):
            return None, None
        return params, y_pred
    except Exception:
        return None, None


def score(y, y_pred, n_params):
    n = len(y)
    resid = y - y_pred
    ss_res = float(np.sum(resid**2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2     = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    # Adjusted R^2 (penalises extra params)
    if n - n_params - 1 > 0:
        adj_r2 = 1.0 - (1.0 - r2) * (n - 1) / (n - n_params - 1)
    else:
        adj_r2 = float("nan")
    rmse = float(np.sqrt(ss_res / n))
    # Gaussian-likelihood AIC (lower = better)
    # AIC = n*ln(SSE/n) + 2k
    aic  = n * np.log(ss_res / n + 1e-12) + 2 * n_params
    return r2, adj_r2, rmse, aic


# -----------------------------------------------------------------------
# Run all models for every sensor
# -----------------------------------------------------------------------
def analyse(data, n_shades):
    present = sorted(data.keys())
    # Lightness axis: shade 0 = 0 % (black), shade N-1 = 100 % (white)
    x_all = np.array([i * 100.0 / (n_shades - 1) for i in range(n_shades)])
    x = np.array([x_all[sid] for sid in present])

    # Per-shade means as the fitted y (same convention as transfer_function.py)
    means = np.full((len(present), NUM_SENSORS), np.nan)
    for row, sid in enumerate(present):
        means[row] = np.array(data[sid]).mean(axis=0)

    # results[model_name][sensor_k] = dict of params + scores
    results = {m[0]: [None] * NUM_SENSORS for m in MODELS}
    predictions = {m[0]: [None] * NUM_SENSORS for m in MODELS}

    for k in range(NUM_SENSORS):
        y = means[:, k]
        for name, fn, guess_fn, n_params, _label in MODELS:
            p0 = guess_fn(x, y)
            params, y_pred = fit_one(fn, p0, x, y)
            if params is None:
                results[name][k]     = None
                predictions[name][k] = None
                continue
            r2, adj_r2, rmse, aic = score(y, y_pred, n_params)
            results[name][k] = {
                "params":  params,
                "r2":      r2,
                "adj_r2":  adj_r2,
                "rmse":    rmse,
                "aic":     aic,
                "n_params": n_params,
            }
            predictions[name][k] = y_pred

    return {
        "x":           x,
        "means":       means,
        "raw":         {sid: np.array(data[sid]) for sid in present},
        "present":     present,
        "lightness":   x_all,
        "results":     results,
        "predictions": predictions,
        "n_shades":    n_shades,
    }


# -----------------------------------------------------------------------
# Reporting
# -----------------------------------------------------------------------
def print_report(ana):
    results = ana["results"]
    print("\n" + "=" * 78)
    print(" Per-sensor regression comparison")
    print("=" * 78)
    print(f" Models tested : {', '.join(m[0] for m in MODELS)}")
    print(f" Shades fitted : {len(ana['present'])}")
    print()

    # Per-sensor tables
    for k in range(NUM_SENSORS):
        print(f"  Sensor {k} (DN{k+1}):")
        print(f"    {'model':<12} {'R²':>8} {'adjR²':>8} {'RMSE':>9} {'AIC':>9}  params")
        rows = []
        for name, _fn, _g, _np, _lbl in MODELS:
            r = results[name][k]
            if r is None:
                rows.append((name, float("-inf"), "FIT FAILED"))
                continue
            rows.append((
                name, r["aic"],
                f"    {name:<12} {r['r2']:>8.4f} {r['adj_r2']:>8.4f} "
                f"{r['rmse']:>9.2f} {r['aic']:>9.1f}  "
                f"{np.array2string(r['params'], precision=3, separator=', ')}"
            ))
        # sort by AIC
        rows.sort(key=lambda t: t[1])
        for _name, _aic, line in rows:
            print(line)
        # Summary pick
        best = min(
            ((name, results[name][k]) for name, *_ in MODELS if results[name][k] is not None),
            key=lambda t: t[1]["aic"],
        )
        print(f"    -> best-by-AIC: {best[0]} "
              f"(R²={best[1]['r2']:.4f}, RMSE={best[1]['rmse']:.2f})")
        print()

    # Aggregate table: pick winner per metric
    print("  Aggregate (best model per sensor):")
    print(f"    {'sensor':>6}  {'best(AIC)':<12}  {'best(adjR²)':<12}")
    for k in range(NUM_SENSORS):
        by_aic = min(
            ((name, results[name][k]) for name, *_ in MODELS if results[name][k] is not None),
            key=lambda t: t[1]["aic"],
        )[0]
        by_adj = max(
            ((name, results[name][k]) for name, *_ in MODELS if results[name][k] is not None),
            key=lambda t: t[1]["adj_r2"],
        )[0]
        print(f"    {k:>6}  {by_aic:<12}  {by_adj:<12}")
    print()


# -----------------------------------------------------------------------
# Overlay plot
# -----------------------------------------------------------------------
def save_plot(ana, input_path):
    x_all     = ana["lightness"]
    results   = ana["results"]
    raw       = ana["raw"]
    means     = ana["means"]
    present   = ana["present"]
    x_pts     = np.array([x_all[sid] for sid in present])
    x_smooth  = np.linspace(0.01, 100, 400)

    stem  = os.path.splitext(os.path.basename(input_path))[0]
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    fig, axes = plt.subplots(1, NUM_SENSORS, figsize=(4.2 * NUM_SENSORS, 4.6),
                             sharex=True, sharey=False)
    fig.suptitle(
        f"Regression-model comparison — {ana['n_shades']} shades — {stamp}",
        fontsize=13,
    )

    colours = {
        "linear":      "tab:red",
        "quadratic":   "tab:purple",
        "cubic":       "tab:brown",
        "exponential": "tab:green",
        "logarithmic": "tab:olive",
        "power":       "tab:cyan",
        "sigmoid":     "black",
    }

    for k in range(NUM_SENSORS):
        ax = axes[k]

        # Raw scatter
        for sid, rows in raw.items():
            xs = np.full(rows.shape[0], x_all[sid])
            ax.scatter(xs, rows[:, k], s=8, alpha=0.25,
                       color="tab:blue", edgecolors="none")

        # Per-shade means
        ax.plot(x_pts, means[:, k], marker="o", color="tab:orange",
                linewidth=1.2, label="mean / shade", zorder=5)

        # Fitted curves
        for name, fn, _g, _np, label in MODELS:
            r = results[name][k]
            if r is None:
                continue
            y_smooth = fn(x_smooth, *r["params"])
            lw = 2.2 if name == "sigmoid" else 1.2
            ls = "-" if name == "sigmoid" else "--"
            ax.plot(x_smooth, y_smooth, ls,
                    color=colours[name], linewidth=lw,
                    label=f"{name} (R²={r['r2']:.3f})")

        ax.set_title(f"Sensor {k} (DN{k+1})")
        ax.set_xlabel("Lightness (%)")
        if k == 0:
            ax.set_ylabel("Raw ADC (0–1023)")
        ax.set_xlim(-5, 105)
        ax.grid(True, alpha=0.3)
        if k == NUM_SENSORS - 1:
            ax.legend(loc="upper right", fontsize=7)

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    os.makedirs(DATA_DIR, exist_ok=True)
    out = os.path.join(DATA_DIR, f"{stem}_regression_comparison.png")
    plt.savefig(out, dpi=120)
    print(f"Saved plot: {out}")


# -----------------------------------------------------------------------
# Residual plot for the linear fit only — highlights the nonlinearity
# -----------------------------------------------------------------------
def save_residual_plot(ana, input_path):
    x_all     = ana["lightness"]
    results   = ana["results"]
    means     = ana["means"]
    present   = ana["present"]
    x_pts     = np.array([x_all[sid] for sid in present])

    stem  = os.path.splitext(os.path.basename(input_path))[0]
    fig, axes = plt.subplots(1, NUM_SENSORS, figsize=(4.2 * NUM_SENSORS, 3.2),
                             sharex=True, sharey=False)
    fig.suptitle("Residuals of LINEAR fit vs. lightness  "
                 "(pattern indicates non-linear true response)", fontsize=12)

    for k in range(NUM_SENSORS):
        ax = axes[k]
        r = results["linear"][k]
        if r is None:
            continue
        a, b = r["params"]
        y_hat = a * x_pts + b
        resid = means[:, k] - y_hat
        ax.axhline(0, color="grey", linewidth=0.8)
        ax.plot(x_pts, resid, "o-", color="tab:red", linewidth=1.2)
        ax.set_title(f"Sensor {k}  (R²={r['r2']:.3f})")
        ax.set_xlabel("Lightness (%)")
        if k == 0:
            ax.set_ylabel("raw − linear fit  (ADC)")
        ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.93])
    out = os.path.join(DATA_DIR, f"{stem}_linear_residuals.png")
    plt.savefig(out, dpi=120)
    print(f"Saved plot: {out}")


# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
def main():
    path     = resolve_input_path(sys.argv)
    n_shades = parse_shade_count(path)
    print(f"Input file : {path}")
    print(f"Shade count: {n_shades}")

    data = load_data(path)
    ana  = analyse(data, n_shades)
    print_report(ana)
    save_plot(ana, path)
    save_residual_plot(ana, path)


if __name__ == "__main__":
    main()
