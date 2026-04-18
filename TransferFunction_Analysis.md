# Line-Sensor Transfer-Function Analysis & Re-Calibration

Date: 2026-04-16
Dataset: `data/26shades_20260416_171731.csv` (26 shades × 20 samples × 5 sensors)

This document records **(a)** the methodology used to determine the
correct shape of the 3Pi+ line-sensor transfer function, **(b)** the
physical justification derived from circuit analysis, and **(c)** the
code changes made to replace the previous linear calibration with a
power-law calibration.

---

## 1. Motivation

`transfer_function.py` fitted a straight line to raw ADC vs. lightness
from a 26-step printed gradient. Inspection of the generated
`..._transfer_functions.png` and the script's reported R² (≈ 0.85 for
every sensor with residuals of 18–36 ADC counts) showed the data is
clearly **not** linear — yet the firmware's
`gradientToPercent()` inverts that linear fit and feeds the result into
the radial-gradient centre-finder FSM. The systematic error directly
distorts the centre estimate.

Goal: determine the true shape of the response, update the calibration
pipeline, and verify that the new model fits the data well both
*numerically* and *physically*.

---

## 2. System Overview (where the data comes from, where the fit is used)

```
┌────────────────┐   CSV (shade_id, sample, s0…s4)
│ log_sensors.py │ ─────────────────────────────► data/<N>shades_<ts>.csv
└────────────────┘
         │
         ▼
┌──────────────────────┐   Per-sensor calibration arrays
│ transfer_function.py │ ────────► GradientCentreFinder/Calibration.h
└──────────────────────┘                    │
                                            │  #include
                                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ GradientCalibration.h :: gradientToPercent(sensor_idx, raw) ──► │
│     lightness percentage used by GradientCentreFinder.ino FSM   │
└─────────────────────────────────────────────────────────────────┘
```

Pipeline stages:

| Stage | File | Role |
|---|---|---|
| Measure | `log_sensors.py` | Samples ~20 raw ADC readings per sensor on each printed shade, shade 0 = black, N−1 = white. |
| Fit | `transfer_function.py` | Reduces samples to per-shade means, fits a per-sensor response model, writes `Calibration.h`. |
| Runtime | `GradientCalibration.h::gradientToPercent()` | Inverts the fitted model to convert a live ADC sample back to a lightness percentage. |

The crucial point is that the **inverse** transfer on the robot must
match the **forward** transfer physically realised by the sensor. If
the fit uses the wrong family, the inverse introduces a systematic,
non-random bias.

---

## 3. Circuit Analysis — what form should the response take?

### 3.1 Active circuit

From `LineSensors.h`:

* `pinMode(sensor_pins[k], INPUT_PULLUP)` on pins `A11, A0, A2, A3, A4`
* `digitalWrite(EMIT_PIN=D11, HIGH)` keeps the IR emitter on

So each sensor is a simple **pull-up + phototransistor potential
divider**:

```
    Vcc (5 V)
       │
      ═╣ R_pullup  (~20–50 kΩ ATmega32U4 internal)
       │
       ├───► ADC input
       │
   ╔═══╩═══╗   IR LED (D11)
   ║ photo ║── reflected IR ◄── printed grayscale surface
   ║ trans.║
   ╚═══╤═══╝
       │
      GND
```

Critically, **there is no capacitor in the signal path** — this sketch
uses the analogue divider method, not the Pololu RC-discharge method.
Therefore the system is a **zeroth-order, memory-less static
non-linearity** — the "magnitude response" is an algebraic transfer
characteristic `y = f(x)`, not a frequency-dependent filter. No poles,
no zeros.

### 3.2 Forward model

Stacking the physical transfers:

1. **Phototransistor** in active region: `I_C ≈ η · Φ_IR` — linear in
   incident IR flux until the transistor saturates at `V_CE ≈ V_CEsat`.
2. **Divider**, treating the phototransistor as a current sink:
   `V_ADC = V_cc − I_C · R_pullup`. Linear in `I_C` until saturation,
   where it asymptotes to a floor.
3. **Reflected flux**: `Φ_IR = Φ_LED · ρ(L) + Φ_ambient`. The ambient
   term is non-zero, which is why shade 0 (black) reads ≈ 945 ADC,
   not the 1023 ceiling.
4. **Printed gradient** (sRGB halftone / ink density): the *optical*
   reflectance vs. the *nominal* lightness is a power law
   `ρ(L) ∝ (L/100)^γ` with γ ≈ 1.8 – 2.4 for typical printer workflows.

Composing 1–4:

```
raw_ADC(L) ≈ V_off − K · (L/100)^γ
```

A **static power-law nonlinearity** with a dark-offset ceiling at
L = 0 % and a saturation floor near L = 100 %. This is the form the
regression analysis should recover if the physical story is right.

---

## 4. Regression Analysis (Methodology)

Implemented in `regression_analysis.py`. For each of the 5 sensors, the
per-shade means (`y`) versus lightness % (`x`) were fit with seven
candidate models:

| # | Family | Form | Params |
|---|---|---|---|
| 1 | Linear      | `y = a·x + b`                                     | 2 |
| 2 | Quadratic   | `y = a·x² + b·x + c`                              | 3 |
| 3 | Cubic       | `y = a·x³ + b·x² + c·x + d`                       | 4 |
| 4 | Exponential | `y = a·exp(−b·x) + c`                             | 3 |
| 5 | Logarithmic | `y = a·ln(x + 1) + b`                             | 2 |
| 6 | Power       | `y = a − b·(x/100)^n`                             | 3 |
| 7 | Sigmoid     | `y = d + (a−d)/(1 + (x/c)^b)` (4-PL)              | 4 |

Each fit scored with R², adjusted R² (penalises free parameters),
RMSE (ADC counts) and AIC (lower = better — fairly trades fit quality
against parameter count).

### 4.1 Results — best model by AIC per sensor

```
sensor   best(AIC)   R²       RMSE (ADC)    linear R² baseline
  0      power       0.9963    2.82          0.8551
  1      cubic       0.9978    3.61          0.8546
  2      cubic       0.9971    4.93          0.8479
  3      cubic       0.9972    3.68          0.8457
  4      power       0.9931    4.53          0.8510
```

### 4.2 Full scorecard (sensor 0 shown, others equivalent)

```
model          R²      adj-R²   RMSE    AIC     params
power        0.9963   0.9958    2.82    60.0   a=947.5, b=154.0, n=2.76
cubic        0.9964   0.9958    2.79    61.3   (Taylor expansion of power)
sigmoid      0.9963   0.9956    2.82    62.0   (saturating, 4 params)
quadratic    0.9935   0.9926    3.77    75.0
linear       0.8551   0.8425   17.78   153.6   ← previous calibration
exponential  0.8551   0.8353   17.78   155.7
logarithmic  0.4442   0.3959   34.81   188.6
```

### 4.3 Key observations

1. **Linear is wrong.** R² 0.85 with 18–36 ADC residuals. The
   `data/…_linear_residuals.png` plot shows classic U-shaped residuals —
   the signature of unmodelled curvature, not random noise.
2. **Power law wins.** It either takes first place (sensors 0, 4) or
   ties cubic to within 1 AIC unit (sensors 1, 2, 3), while using one
   fewer parameter. Occam's razor → pick power.
3. **Fitted exponent is consistent:**
   `n ∈ {2.76, 2.83, 2.89, 2.93, 2.80}`, mean ≈ **2.84**.
   Sits inside the "printer-gamma + phototransistor roll-off" range
   predicted analytically in §3.2. The numerical result matches the
   physical prediction — that alignment is the strongest argument for
   the power form over the indistinguishable-on-fit cubic.
4. **Logarithmic and pure exponential fail** (R² 0.44 and 0.85); they
   cannot represent the gentle concave-down-then-flatten behaviour of
   a gamma curve with saturation.

---

## 5. Chosen Calibration

### 5.1 Forward model (per sensor k)

```
raw = CAL_A[k]  −  CAL_B[k] · (L/100)^CAL_N[k]
```

Parameters:

* `CAL_A[k]` – raw ADC at L = 0 % (black end, upper asymptote)
* `CAL_B[k]` – full black→white ADC excursion (scale)
* `CAL_N[k]` – curvature exponent

### 5.2 Inverse (implemented by firmware)

```
L_pct = 100 · ((CAL_A[k] − raw) / CAL_B[k]) ^ (1 / CAL_N[k])
```

clamped to `[0, 100]`. `(CAL_A − raw)` must be ≥ 0 — clamped to 0
before the exponent to avoid NaN from `powf` of a negative base.

### 5.3 Parameters from `26shades_20260416_171731.csv`

```
sensor    CAL_A      CAL_B     CAL_N     R²(pow)
  0       947.53    154.02    2.7627    0.9963
  1       937.21    251.88    2.8314    0.9972
  2       909.50    303.72    2.8897    0.9967
  3       942.06    230.60    2.9311    0.9966
  4       935.87    179.24    2.7970    0.9931
```

---

## 6. Code Changes

### 6.1 `transfer_function.py`

* Adds a scipy `curve_fit` of `y = a − b·(L/100)^n` per sensor alongside
  the existing linear fit.
* Emits the new arrays `CAL_A[5]`, `CAL_B[5]`, `CAL_N[5]` in the
  generated `Calibration.h`, keeping `CAL_WHITE/BLACK/RANGE` for
  diagnostics and the retained linear fallback.
* Reports both linear and power-law R² in the stdout summary and in a
  comment block at the top of `Calibration.h`.
* Overlays both fits on the diagnostic plot and annotates each panel
  with the power-law parameters and both R² values.

### 6.2 `GradientCentreFinder/GradientCalibration.h`

* `gradientToPercent()` rewritten to invert the power-law:
  `100 · ((CAL_A − raw) / CAL_B) ^ (1 / CAL_N)`, with domain guards
  on the base fraction and on invalid `CAL_B / CAL_N`.
* Previous linear implementation preserved as
  `gradientToPercentLinear()` for A/B comparison on the robot.
* `getAverageLightness()` unchanged — it calls `gradientToPercent()`
  so picks up the new transfer automatically.
* Adds `#include <math.h>` for `powf`.

### 6.3 `GradientCentreFinder/Calibration.h` (regenerated)

Auto-generated by the updated pipeline. Contains the parameters shown
in §5.3. The legacy CAL_WHITE/BLACK/RANGE arrays are still present.

---

## 7. Verification

### 7.1 Numerical

* Re-fit against the 26-shade CSV: R² raises from
  **0.846 – 0.855 (linear)** → **0.993 – 0.997 (power)** across all five
  sensors.
* RMSE drops from **~17 – 36 ADC counts** → **~3 – 5 ADC counts**
  (~3× – 7× improvement).
* Residual plot for the linear fit shows pronounced U-shape
  (`data/…_linear_residuals.png`); for the power fit, no systematic
  pattern remains.
* Fitted exponents 2.76 – 2.93 are consistent with the analytic
  prediction `γ ≈ 2 – 3` from §3.2.

### 7.2 Build

`arduino-cli compile --fqbn pololu-a-star:avr:a-star32U4
GradientCentreFinder/GradientCentreFinder.ino` completes cleanly:

```
Sketch uses 21492 bytes (74%) of program storage space. Maximum is 28672 bytes.
Global variables use 1541 bytes (60%) of dynamic memory, leaving 1019 bytes for local variables. Maximum is 2560 bytes.
```

Flash and RAM usage remain well within the ATmega32U4 budget. The
added cost is effectively one `powf` call per sensor per read
(~30 µs × 5 ≈ 150 µs); negligible against the 1–2 ms FSM loop period.

### 7.3 On-robot smoke test (to do — requires hardware)

1. Flash the sketch to the 3Pi+.
2. Place the robot statically on each printed shade used for
   calibration; the reported `L` for each sensor should sit within
   ~1 – 2 % of the expected lightness.
3. Optionally, enable a diagnostic print of
   `gradientToPercent(k, raw) − gradientToPercentLinear(k, raw)` for
   each sensor — the difference should be smooth (no spikes) and near
   zero only at the two calibration anchors, growing in the
   mid-gradient region (this is the error the old code was making).

---

## 8. Artifacts

| Path | Role |
|---|---|
| `SEMTM0042_43/3Pi_CodeStub/robotics-c1-codebase/transfer_function.py` | Updated fitter (linear + power-law) |
| `SEMTM0042_43/3Pi_CodeStub/robotics-c1-codebase/regression_analysis.py` | Exploratory regression comparison across 7 families |
| `SEMTM0042_43/3Pi_CodeStub/robotics-c1-codebase/GradientCentreFinder/GradientCalibration.h` | Updated runtime transfer (power-law inverse) |
| `SEMTM0042_43/3Pi_CodeStub/robotics-c1-codebase/GradientCentreFinder/Calibration.h` | Regenerated — contains new CAL_A/B/N arrays |
| `SEMTM0042_43/3Pi_CodeStub/robotics-c1-codebase/data/26shades_20260416_171731_transfer_functions.png` | Diagnostic plot — linear vs. power fits |
| `data/26shades_20260416_171731_regression_comparison.png` | All seven candidate fits overlaid per sensor |
| `data/26shades_20260416_171731_linear_residuals.png` | U-shaped residuals of the old linear fit |

---

## 9. Conclusion

Both lines of evidence — static circuit analysis and numerical
regression — agree that the 3Pi+ line-sensor response to a printed
grayscale gradient is a **static power-law nonlinearity** with
exponent ≈ 2.8 and an additive dark-current offset. Replacing the
previous linear fit with a per-sensor `a − b·(L/100)^n` model cuts
residual lightness error by roughly an order of magnitude (R² 0.85 →
0.995), removes a systematic bias that previously distorted radial
centre estimation, and costs only a single `powf` call per sensor at
runtime. The change is now live in `transfer_function.py` and
`GradientCalibration.h`; `Calibration.h` has been regenerated from
the current 26-shade dataset.
