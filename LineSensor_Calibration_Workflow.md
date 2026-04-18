# Line Sensor Calibration — Workflow Guide

End-to-end procedure for characterising the 3Pi+ line sensors against a
printed grayscale shade card and producing a `Calibration.h` header for
use in the robot firmware.

All paths are relative to:
`SEMTM0042_43/3Pi_CodeStub/robotics-c1-codebase/`

---

## 0. What You Need

### Hardware
- Pololu 3Pi+ 32U4 robot, fully charged
- USB cable (A ↔ micro-B)
- Printed shade card — a row of grayscale patches from **black (darkest)** to **white (lightest)**. Any N ≥ 3 works; 10–25 patches is typical.
- A flat, well-lit surface

### Software
- `arduino-cli` with the `pololu-a-star:avr` core installed
- Python 3.8+
- Python packages: `pyserial`, `numpy`, `matplotlib`
  ```bash
  pip install pyserial numpy matplotlib
  ```

### Files involved
| File | Role |
|---|---|
| `LineSensorTest/LineSensorTest.ino` | Robot-side capture firmware |
| `log_sensors.py` | Host-side serial logger |
| `transfer_function.py` | Calibration fitter & header generator |
| `3Pi_CodeStub/Calibration.h` | **Output** — regenerated each run |
| `data/<N>shades_<timestamp>.csv` | **Output** — raw ADC samples |
| `data/<stem>_transfer_functions.png` | **Output** — diagnostic plot |

---

## 1. Flash the Calibration Firmware

From the repo root:

```bash
# Compile
arduino-cli compile \
  --fqbn pololu-a-star:avr:a-star32U4 \
  LineSensorTest/LineSensorTest.ino

# Upload — replace the port glob with your actual port if needed
arduino-cli upload \
  -p /dev/tty.usbmodem* \
  --fqbn pololu-a-star:avr:a-star32U4 \
  LineSensorTest/LineSensorTest.ino
```

On upload the robot beeps once (440 Hz). Over serial it prints a banner
describing the CSV format and asks you to place it on shade 0 (black).

---

## 2. Start the Logger

In a second terminal, still at the repo root:

```bash
# Auto-detect port, shade count inferred at the end
python log_sensors.py

# OR pre-declare how many shades you plan to capture
python log_sensors.py --shades 25

# OR force a specific port
python log_sensors.py /dev/tty.usbmodem14201 --shades 25
```

The logger opens a temporary file `data/_tmp_<timestamp>.csv` and streams
everything arriving from the robot into it, flushing after every line so
Ctrl+C cannot lose data. Comment lines from the robot are shown in grey;
shade-complete banners are shown in yellow.

---

## 3. Capture Data — One Patch at a Time

Capture in order from **darkest (shade 0)** to **lightest (shade N−1)**:

1. Place the robot flat on the shade patch. The **centre sensor (DN3)**
   must sit over the patch; keep the robot stationary.
2. Press **Button A**.
   - Short 660 Hz beep = capture started.
   - The robot emits 20 CSV rows, one sample every ~5 ms.
   - Higher 880 Hz beep = capture done, `shade_id` has incremented.
3. Lift the robot, move it to the next (lighter) patch, repeat.

Each press produces exactly 20 rows tagged with the current `shade_id`,
followed by a `# Shade N complete.` banner. The logger highlights this
banner so you can keep track without reading the CSV.

### What each CSV row looks like
```
shade_id, sample, s0, s1, s2, s3, s4
0,0,978,965,941,955,970
0,1,977,963,940,954,969
…
0,19,979,966,942,956,971
# Shade 0 complete. Place robot on next shade and press Button A.
1,0,821,803,789,812,830
…
```

---

## 4. Stop the Logger

When the last (white) patch is done, press **Ctrl+C** in the logger
terminal. The logger:

1. Closes the temp file.
2. Counts unique `shade_id` values in it.
3. Renames it to `data/<N>shades_<timestamp>.csv`.

Expected output:
```
──────────────────────────────────────────────────────────
Logging stopped.
  Data rows saved : 500
  Comment lines   : 26
  Shades captured : 25
  Saved as        : data/25shades_20260416_143022.csv
```

If `--shades` was given and doesn't match, a `NOTE:` is printed but the
file is still saved.

---

## 5. Fit the Transfer Functions

```bash
# Uses the newest Nshades_*.csv in data/
python transfer_function.py

# Or pass an explicit file
python transfer_function.py data/25shades_20260416_143022.csv
```

The script:

1. Parses `N` from the filename.
2. Groups the CSV by `shade_id`, computes the mean ADC per sensor per shade.
3. Fits `lightness(%) → mean_ADC` via least-squares for each sensor
   (`lightness = 0%` at shade 0 / black, `100%` at shade N−1 / white).
4. Anchors `CAL_BLACK[k]` / `CAL_WHITE[k]` from the black and white means.
5. Writes `3Pi_CodeStub/Calibration.h` (overwriting any previous version).
6. Saves `data/<stem>_transfer_functions.png` and displays the plot.

### Console summary
```
Input file : data/25shades_20260416_143022.csv
Shade count: 25

Per-sensor summary:
   k    white    black    range    slope      R²
   0   179.10   969.80   790.70   -7.908   1.000
   1   174.10   964.45   790.35   -7.901   1.000
   …
Wrote 3Pi_CodeStub/Calibration.h
Saved plot: data/25shades_20260416_143022_transfer_functions.png
```

### Sanity checks to confirm before moving on
- `range` column should be **> 50 ADC units** on every sensor. A warning
  is printed if not — redo the capture with better lighting or a wider
  shade range if you see this.
- R² should be close to 1.0 (≥ ~0.95). Much lower means the sensor's
  response is non-linear over your shade set or the capture had motion.
- In the plot each panel should show a monotone decline from black (high
  ADC) on the left to white (low ADC) on the right.

---

## 6. Using `Calibration.h` in Firmware

`3Pi_CodeStub/Calibration.h` exposes three arrays:

```c
const float CAL_WHITE[5];   // mean ADC on white surface
const float CAL_BLACK[5];   // mean ADC on black surface
const float CAL_RANGE[5];   // CAL_BLACK - CAL_WHITE (pre-computed)
```

In downstream code (future work — **not** part of this calibration task):

```c
#include "Calibration.h"

float darkness = (readings[k] - CAL_WHITE[k]) / CAL_RANGE[k];
if (darkness < 0.0f) darkness = 0.0f;
if (darkness > 1.0f) darkness = 1.0f;
// 0.0 = white, 1.0 = black — directly comparable across sensors
```

`LineSensors.h` itself is **unchanged** by this workflow; wiring these
arrays into `LineSensors_c::calcCalibratedADC()` is a separate task.

---

## 7. Re-running the Calibration

The entire pipeline is idempotent and hot-swappable:

- Re-flash the firmware only if you change `LineSensorTest.ino`.
- Run `log_sensors.py` → capture → Ctrl+C for every new data set.
  Each run creates a new `data/<N>shades_<timestamp>.csv`; nothing is
  overwritten.
- Run `transfer_function.py` against any historical CSV at any time.
  Each run overwrites `3Pi_CodeStub/Calibration.h` and saves a new plot.
- `N` is parsed from the filename, so a 10-shade card and a 25-shade
  card work with the same script — no code edits needed.

---

## 8. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Logger prints `Waiting for /dev/tty.usbmodem* …` forever | Robot not connected, or port name wrong. Plug in USB, or pass the port explicitly as the first argument. |
| Upload fails with "device busy" | A serial monitor or `log_sensors.py` is holding the port. Close it first. |
| Logger says `No shade data captured — leaving temp file` | You Ctrl+C'd before pressing Button A even once. Temp file is kept; rerun the capture. |
| `transfer_function.py` errors: `could not parse shade count from filename` | Input filename doesn't match `<N>shades_…csv`. Rename or pass a correct file. |
| `WARNING: sensor k has a small black-white range` | Sensor barely responded — check that the IR LED is on, the shade patches aren't shiny, and there is good contrast between shade 0 and shade N−1. |
| R² low, plot non-monotone | Robot moved during capture, or a patch was captured out of order. Re-run, darkest-first, holding the robot still. |
| Button A seems to do nothing | 200 ms debounce — hold slightly longer or release and press again. Check the 660 Hz "start" beep is heard. |

---

## 9. Quick-Reference Command Cheat Sheet

```bash
# 1. Flash calibration firmware
arduino-cli compile --fqbn pololu-a-star:avr:a-star32U4 LineSensorTest/LineSensorTest.ino
arduino-cli upload -p /dev/tty.usbmodem* --fqbn pololu-a-star:avr:a-star32U4 LineSensorTest/LineSensorTest.ino

# 2. Start logger (optionally pre-declare shade count)
python log_sensors.py --shades 25

# 3. Capture each patch (black → white): Button A, wait for high beep, move, repeat
# 4. Stop with Ctrl+C when the white patch is done

# 5. Fit & generate Calibration.h + plot
python transfer_function.py
```
