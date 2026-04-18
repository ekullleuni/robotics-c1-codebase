# Line Sensor Calibration — Implementation Plan

> **For Opus 4.6 with no prior context.** This document is self-contained.
> All file paths are relative to the repo root:
> `SEMTM0042_43/3Pi_CodeStub/robotics-c1-codebase/`

---

## 1. Background & Goal

### Hardware
The robot is a **Pololu 3Pi+ 32U4** (ATmega32U4, 5 V, Arduino-compatible).
It has **5 downward-facing IR reflectance sensors** (DN1–DN5):
- Pin mapping (already in `3Pi_CodeStub/LineSensors.h`):
  `{ A11, A0, A2, A3, A4 }` — left to right across the robot's underside
- Shared IR LED enable on **pin 11** (HIGH = IR on); all sensors share one enable line
- Sensors are read via `analogRead()` → 10-bit value **0–1023**
- **Higher ADC value = darker surface** (less reflection)
- Internal pull-up must be enabled on each sensor pin (`INPUT_PULLUP`) to form the potential divider

### Application context
The final robot behaviour is:
1. Drive toward the centre of a printed radial gradient (black centre, white perimeter)
2. Direction: compare the two **edge sensors** (DN1 left, DN5 right) — turn towards the darker one
3. Arrival: the **centre sensor** (DN3) exceeds a darkness threshold

For this to work, all 5 sensors must produce **normalised, comparable readings** in [0 = white, 1 = black]. Raw ADC values cannot be compared directly because individual LED/photodiode tolerances create per-sensor offsets. A **per-sensor linear transfer function** maps raw ADC → normalised darkness.

### What this plan implements
1. **Simplified `LineSensorTest.ino`** — stripped firmware: place robot on shade → press Button A → 20 readings emitted over serial → repeat for all shades
2. **Updated `log_sensors.py`** — captures serial output to a file named `{N}shades_{timestamp}.csv`
3. **New `transfer_function.py`** — loads any `{N}shades_*.csv` file, fits per-sensor linear calibration, generates a ready-to-include `Calibration.h` Arduino header and diagnostic plots

---

## 2. Files to Create / Modify

| Action | Path |
|--------|------|
| **Rewrite** | `LineSensorTest/LineSensorTest.ino` |
| **Modify** | `log_sensors.py` |
| **Create** | `transfer_function.py` |

No other files should be touched.

---

## 3. File 1 — `LineSensorTest/LineSensorTest.ino`

### What to keep from the existing file
- `#include "LineSensors.h"` (the only dependency needed)
- `BUZZER_PIN`, `BUTTON_A_PIN` definitions
- `LineSensors_c line_sensors` instance
- `buttonPressed()` debounce helper and `buttonA()` shorthand
- `printSensorCSV()` helper

### What to remove entirely
- All `#include` lines except `LineSensors.h`
- All motor, PID, encoder, kinematics, speed-measurement variables and functions
- All Phase 2 and Phase 3 state-machine cases and variables
- The `BUTTON_B_PIN` definition and `buttonB()` function (Button B is not used)
- `updatePID()`, `updatePose()`, `measureSpeeds()`, `angleDiff()`, `distTo()`,
  `setDemand()`, `stopMotors()`, `startRotation()`, `updateRotation()`,
  `startReverse()`, `updateReverse()` functions

### Remaining state machine — two states only

```
IDLE  ──── Button A pressed ────▶  READING (takes 20 samples) ──▶  IDLE
```

```cpp
enum TestPhase { TEST_IDLE, TEST_READING };
TestPhase test_phase = TEST_IDLE;
```

### Variables needed
```cpp
int shade_id     = 0;   // increments after each capture; starts at 0
int sample_count = 0;   // 0..19 during READING
```

### Configuration defines
```cpp
#define SAMPLES_PER_SHADE  20    // Readings per shade press
#define SAMPLE_DELAY_MS     5    // ADC settling time between samples (ms)
```

### `setup()` — what it must do
1. `pinMode(BUZZER_PIN, OUTPUT)`
2. `pinMode(BUTTON_A_PIN, INPUT_PULLUP)`
3. `line_sensors.initialiseForADC()` — enables IR LED, configures sensor pins
4. `Serial.begin(115200)`; `delay(1500)` — wait for USB CDC to enumerate
5. Print welcome banner (comments prefixed `#` so the logger can distinguish them):
   ```
   # LINE SENSOR SHADE CALIBRATION
   # ==============================
   # shade_id 0 = BLACK (darkest patch)
   # Increasing shade_id = progressively lighter
   # Final shade = WHITE (lightest patch)
   # Place robot on shade, press BUTTON A to capture 20 readings.
   # CSV format: shade_id, sample, s0, s1, s2, s3, s4
   #
   # Ready. Place robot on shade 0 (BLACK) and press Button A.
   ```
6. Short beep: `tone(BUZZER_PIN, 440, 100)`

### `loop()` — behaviour

```
Every loop iteration:
  - Poll buttonA() with debounce

  switch(test_phase):

  case TEST_IDLE:
    if buttonA pressed:
      sample_count = 0
      test_phase   = TEST_READING
      tone(BUZZER_PIN, 660, 50)

  case TEST_READING:
    line_sensors.readSensorsADC()
    print CSV line: shade_id, sample_count, s0, s1, s2, s3, s4
    sample_count++

    if sample_count < SAMPLES_PER_SHADE:
      delay(SAMPLE_DELAY_MS)          // ADC settling
    else:
      // Shade complete
      shade_id++
      tone(BUZZER_PIN, 880, 100)
      print: "# Shade N complete. Place robot on next shade and press Button A."
      test_phase = TEST_IDLE
```

### CSV output format (data rows — no `#` prefix)
```
shade_id, sample, s0, s1, s2, s3, s4
```
- `shade_id`: integer, starts at 0 (black), increments after every 20-sample capture
- `sample`: integer 0–19
- `s0`–`s4`: raw 10-bit ADC integers (0–1023), no decimal point

**Example:**
```
0,0,978,965,941,955,970
0,1,977,963,940,954,969
...
0,19,979,966,942,956,971
# Shade 0 complete. Place robot on next shade and press Button A.
1,0,821,803,789,812,830
```

### Notes on `readSensorsADC()` (from `LineSensors.h`)
The existing implementation calls `initialiseForADC()` every time it reads — this re-enables the IR LED and reconfigures pins on every call. This is safe but slightly wasteful; **do not change `LineSensors.h`**, just call `readSensorsADC()` as-is.

---

## 4. File 2 — `log_sensors.py` (modify existing)

### Change: filename encoding
The current logger produces `data/{timestamp}.csv`. Change this so the filename also encodes the number of shades measured.

**Problem:** the number of shades is not known at the time the file is opened (the user decides when to stop). 

**Solution:** open the file using a temporary name, count unique `shade_id` values at Ctrl+C, then rename.

### Specific changes required

#### Add `--shades` argument (optional)
If the user knows in advance:
```
python log_sensors.py --shades 25
```
If omitted, the shade count is determined from the data at close time.

#### File naming
```python
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
tmp_path  = os.path.join(OUTPUT_DIR, f"_tmp_{timestamp}.csv")
# ... write to tmp_path during logging ...
# On exit:
shade_ids = set()
with open(tmp_path) as f:
    for line in f:
        if line.startswith("#") or not line.strip(): continue
        parts = line.split(",")
        try: shade_ids.add(int(parts[0]))
        except (ValueError, IndexError): pass
n_shades  = len(shade_ids)
final_path = os.path.join(OUTPUT_DIR, f"{n_shades}shades_{timestamp}.csv")
os.rename(tmp_path, final_path)
print(f"Saved as: {final_path}  ({n_shades} shades captured)")
```

#### Print shade progress to terminal
When a `# Shade N complete` comment arrives, print a highlighted progress line so the operator can track status without reading the CSV.

#### Keep everything else the same
- Port auto-detection: unchanged
- Reconnection on USB reset: unchanged
- Ctrl+C handling: unchanged (but add the rename step in `finally`)
- Comment lines echoed in grey: unchanged

---

## 5. File 3 — `transfer_function.py` (create new)

### Purpose
1. Load a `{N}shades_{timestamp}.csv` data file
2. For each of the 5 sensors, compute the mean ADC reading at each shade level
3. Fit a **linear regression** (least squares) mapping `lightness_percent → mean_ADC`
4. Derive calibration coefficients that let the robot compute:
   `normalised_darkness = clamp((adc - white_mean) / (black_mean - white_mean), 0.0, 1.0)`
5. Write a `Calibration.h` Arduino header with the coefficients
6. Save a diagnostic plot

### File naming / hot-swap compatibility
Parse `N` from the filename pattern `(\d+)shades_` using a regex.  
This determines how many shade levels there are and how lightness is computed:
```python
import re, os
m = re.search(r'(\d+)shades_', os.path.basename(filepath))
n_shades = int(m.group(1))
lightness = [i * 100.0 / (n_shades - 1) for i in range(n_shades)]
```
This means the script works identically for 10-shade or 25-shade input files with no code changes.

### Data loading
Parse the CSV, skip `#` comment lines, group rows by `shade_id`:
```python
# Row format: shade_id, sample, s0, s1, s2, s3, s4
import csv, collections, numpy as np
data = collections.defaultdict(list)   # {shade_id: [[s0,s1,s2,s3,s4], ...]}
with open(filepath) as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'): continue
        parts = line.split(',')
        if len(parts) != 7: continue
        try:
            sid = int(parts[0])
            sensors = [int(x) for x in parts[2:7]]
            data[sid].append(sensors)
        except ValueError: continue
```

Validate: assert `len(data) == n_shades` (or warn if shades are missing).

### Per-sensor calibration

For each sensor `k` (0–4):

```python
means_k = [np.mean([row[k] for row in data[sid]]) for sid in range(n_shades)]

# Linear regression: lightness (%) → mean ADC
# np.polyfit returns [slope, intercept]
coeffs_k = np.polyfit(lightness, means_k, 1)   # slope, intercept

# Black and white anchor points (shade 0 = black, shade n_shades-1 = white)
black_mean_k = means_k[0]           # ADC at black (highest value)
white_mean_k = means_k[n_shades-1]  # ADC at white (lowest value)
range_k      = black_mean_k - white_mean_k

# Normalisation formula (for robot firmware):
# normalised_darkness = (adc - white_mean) / range
# = 0.0 at white, 1.0 at black
```

Verify `range_k > 50` (sanity check — if range is tiny, the calibration data is bad). Warn if not.

### Generate `Calibration.h`

Write this file to `3Pi_CodeStub/3Pi_CodeStub/Calibration.h` (overwrite if exists):

```c
/*
 * Calibration.h — AUTO-GENERATED by transfer_function.py
 * Source file: <input filename>
 * Generated  : <timestamp>
 * Shades used: <N>
 *
 * Normalised darkness formula (per sensor k):
 *   float darkness = (readings[k] - CAL_WHITE[k]) / CAL_RANGE[k];
 *   if (darkness < 0.0) darkness = 0.0;
 *   if (darkness > 1.0) darkness = 1.0;
 *
 * 0.0 = white surface, 1.0 = black surface
 */
#ifndef _CALIBRATION_H
#define _CALIBRATION_H

// Mean ADC reading on white surface (lowest ADC = least dark)
const float CAL_WHITE[5] = { <w0>, <w1>, <w2>, <w3>, <w4> };

// Mean ADC reading on black surface (highest ADC = most dark)
const float CAL_BLACK[5] = { <b0>, <b1>, <b2>, <b3>, <b4> };

// Range = CAL_BLACK - CAL_WHITE  (pre-computed for efficiency)
const float CAL_RANGE[5] = { <r0>, <r1>, <r2>, <r3>, <r4> };

#endif
```

Values should be written to 2 decimal places. The downstream `LineSensors_c::calcCalibratedADC()` should be updated (separately, not in this plan) to use these arrays instead of its current min/max/scaling arrays — but that is a **future task**. For now, `Calibration.h` just stores the values.

### Diagnostic plot

5-panel figure (one subplot per sensor), shared x-axis = lightness %:
- Scatter: individual raw ADC readings (small dots, alpha 0.3)
- Line: per-shade mean ADC
- Line: fitted linear regression
- Annotations: R², slope, intercept on each subplot
- Title: `"Sensor Transfer Functions — {N} shades — {timestamp}"`

Save as: `data/{input_stem}_transfer_functions.png`
Also call `plt.show()`.

### CLI interface
```
python transfer_function.py                    # uses most recent file in data/
python transfer_function.py data/25shades_*.csv  # explicit file
```

---

## 6. Workflow (Human Operator Steps)

1. **Flash firmware**: compile and upload the new `LineSensorTest.ino` to the robot
2. **Start logger**: `python log_sensors.py` (or `python log_sensors.py --shades 25`)
3. **Collect data** (repeat for every shade patch, darkest first):
   - Place robot flat on shade patch, centre sensor (DN3) directly over the patch
   - Press **Button A** → robot beeps, emits 20 readings, beeps again
   - Move to next (lighter) patch
4. **Stop logger**: press `Ctrl+C` → file saved as e.g. `data/25shades_20260416_143022.csv`
5. **Analyse**: `python transfer_function.py` → generates `Calibration.h` and plot

---

## 7. Key Constraints for Implementation

### Arduino
- **No `delay()` outside the reading loop** — `SAMPLE_DELAY_MS` inside `TEST_READING` is intentional and acceptable (it is a blocking characterisation sketch, not the main robot firmware)
- **No motor, PID, encoder or kinematics code** — the robot sits still during this test
- **Debounce**: 200 ms is already proven in the existing code; keep it
- **Serial baud**: 115200 — the USB CDC on ATmega32U4 makes the baud rate nominal, but must match the Python logger
- `readSensorsADC()` internally calls `initialiseForADC()` each time — this is fine
- **Do not modify `LineSensors.h`**

### Python
- **Dependencies**: `pyserial`, `numpy`, `matplotlib` — all standard; add to a `requirements.txt` if one exists
- **Python 3.8+** assumed
- **No pandas** — use plain `csv`/list operations for portability
- The logger must be interruptible at any point without losing data (flush after every line — already done in existing code)
- `transfer_function.py` must work if **any subset of shades** is missing (e.g., only 20 of 25 collected) — just skip missing shade IDs in the regression

### Calibration header
- `Calibration.h` lives at `3Pi_CodeStub/3Pi_CodeStub/Calibration.h`
- It is **re-generated** each time `transfer_function.py` runs — do not hand-edit it
- The `#ifndef _CALIBRATION_H` guard must be present to prevent double-inclusion

---

## 8. What NOT to implement

- Do not implement the robot's gradient-following behaviour (that is a separate task)
- Do not modify `LineSensors.h`
- Do not modify the main `3Pi_CodeStub.ino`
- Do not add a Phase 2 or Phase 3 back into `LineSensorTest.ino`
- Do not add `plot_shade.py` modifications — it is superseded by `transfer_function.py` for this workflow
- Do not write a `requirements.txt` unless one already exists

---

## 9. Acceptance Criteria

| # | Criterion |
|---|-----------|
| 1 | `LineSensorTest.ino` compiles without errors or warnings using `arduino-cli --fqbn pololu-a-star:avr:a-star32U4` |
| 2 | After flashing, pressing Button A on the robot emits exactly 20 CSV data rows over serial (verified by terminal) |
| 3 | Each press increments `shade_id` in the output |
| 4 | `log_sensors.py` creates a file matching `data/\d+shades_\d{8}_\d{6}\.csv` after Ctrl+C |
| 5 | `transfer_function.py` runs on the generated CSV with no errors |
| 6 | `Calibration.h` is produced with 5 values each for `CAL_WHITE`, `CAL_BLACK`, `CAL_RANGE` |
| 7 | `CAL_RANGE[k] > 50` for all sensors (sanity: sensors responded to the full black→white range) |
| 8 | The diagnostic plot shows sensible monotone curves (ADC rises from white to black) |
