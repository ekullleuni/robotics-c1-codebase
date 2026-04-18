# Radial Gradient Centre-Finder — Workflow Guide

End-to-end procedure for calibrating, flashing, running, and tuning the
radial gradient centre-finder firmware on a Pololu 3Pi+ 32U4.

All paths in this document are relative to the repo root unless stated
otherwise. The sketch folder is:
`SEMTM0042_43/final code/GradientCentreFinder/`

See `../../RadialGradient_FSM_Plan.md` for the design rationale — this
document covers the *methodology for running the code*, not its design.

---

## 0. What You Need

### Hardware
- Pololu 3Pi+ 32U4 robot, fully charged
- USB cable (A ↔ micro-B)
- **Printed grayscale shade card** — 25 patches from black (shade 0) to
  white (shade 24) — used once per environment for offline calibration.
- **Printed radial gradient map** — pure black at the centre, fading
  continuously to white at the perimeter. Any radial profile that is
  monotone from centre to edge will work.
- A flat, well-lit surface for both.

### Software
- `arduino-cli` with the `pololu-a-star:avr` core installed
- Serial monitor (Arduino IDE, `arduino-cli monitor`, `screen`, …) at
  **9600 baud**
- Python 3.8+ with `pyserial`, `numpy`, `matplotlib` — only needed for
  the offline calibration step (§2).

### Files involved
| File | Role |
|---|---|
| `GradientCentreFinder.ino` | Main sketch — FSM described in §8 |
| `Calibration.h` | **Compile-time** per-sensor anchor arrays used by the transfer function (§2) |
| `GradientCalibration.h` | Thin wrapper — `gradientToPercent()` implemented as `100 * (CAL_BLACK − raw) / CAL_RANGE` |
| `Motors.h` `Encoders.h` `PID.h` `Kinematics.h` `SpeedMeasurement.h` `LineSensors.h` `oled.h` `lcd.h` | Copied verbatim from Part 1 — **do not edit** |

---

## 1. One-Time Overview

This firmware performs **no on-robot calibration routine**. The
per-sensor transfer-function anchors are produced **offline**, saved
into `Calibration.h`, and flashed into the robot as part of the
sketch. That decouples calibration (slow, done rarely) from runs
(fast, repeated).

The end-to-end sequence is:

```
┌─────────────────────────────────────────────────────────────────┐
│ Offline, once per environment (§2)                              │
│   LineSensorTest firmware  → log_sensors.py → transfer_function │
│                            → Calibration.h                      │
└──────────────────────────────┬──────────────────────────────────┘
                               │ copy / point at
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│ Offline, per session (§3)                                       │
│   arduino-cli compile + upload  GradientCentreFinder.ino        │
└──────────────────────────────┬──────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│ On-robot, repeatable (§§4–7)                                    │
│   Button A → placement ack → Button A → run → results → restart │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Generate `Calibration.h` (Offline Calibration)

`Calibration.h` in this folder ships with **placeholder values**. Every
new environment (new map, different lighting, different sensor batch)
requires regenerating it.

Use the pre-existing offline workflow in
`SEMTM0042_43/3Pi_CodeStub/robotics-c1-codebase/`. The full procedure is
documented in that folder's `LineSensor_Calibration_Workflow.md`; a
compressed recap follows.

### 2.1 Capture
1. Flash the capture firmware:
   ```bash
   cd "SEMTM0042_43/3Pi_CodeStub/robotics-c1-codebase"
   arduino-cli compile --fqbn pololu-a-star:avr:a-star32U4 LineSensorTest/LineSensorTest.ino
   arduino-cli upload  -p /dev/tty.usbmodem* --fqbn pololu-a-star:avr:a-star32U4 LineSensorTest/LineSensorTest.ino
   ```
2. Start the host logger:
   ```bash
   python log_sensors.py --shades 25
   ```
3. Place the robot over shade 0 (black). Press **Button A**. The robot
   captures 20 samples; a high beep marks completion. Move to shade 1;
   press A; repeat through shade 24 (white). `Ctrl+C` the logger when
   done — it saves `data/25shades_<timestamp>.csv`.

### 2.2 Fit
```bash
python transfer_function.py
```
This writes `3Pi_CodeStub/Calibration.h` (relative to
`robotics-c1-codebase/`) containing:

```c
const float CAL_WHITE[5] = { … };   // mean ADC on white surface
const float CAL_BLACK[5] = { … };   // mean ADC on black surface
const float CAL_RANGE[5] = { … };   // CAL_BLACK - CAL_WHITE
```

Sanity-check the console summary before continuing:
- `range` > 50 ADC units per sensor.
- `R²` ≥ 0.95 per sensor.
- Plot per sensor is monotone from black (high ADC) to white (low ADC).

### 2.3 Install into the centre-finder sketch
Copy the generated header into this folder, overwriting the
placeholder:

```bash
cp "SEMTM0042_43/3Pi_CodeStub/robotics-c1-codebase/3Pi_CodeStub/Calibration.h" \
   "SEMTM0042_43/final code/GradientCentreFinder/Calibration.h"
```

(Alternatively, edit `CALIB_OUT_PATH` at the top of `transfer_function.py`
to point straight at this folder.)

This is the only per-environment step required before a centre-finding
session. It is **not** repeated between runs within a session.

### 2.4 Verify the transfer function
Optional but recommended: open `Calibration.h` and eyeball the numbers.
`CAL_BLACK[k]` should be noticeably larger than `CAL_WHITE[k]` for every
sensor; their difference equals `CAL_RANGE[k]`.

---

## 3. Flash the Centre-Finder Firmware

Once `Calibration.h` contains real numbers, build and upload from the
repo root:

```bash
# Compile
arduino-cli compile \
  --fqbn pololu-a-star:avr:a-star32U4 \
  "final code/GradientCentreFinder/GradientCentreFinder.ino"

# Upload
arduino-cli upload \
  -p /dev/tty.usbmodem* \
  --fqbn pololu-a-star:avr:a-star32U4 \
  "final code/GradientCentreFinder/GradientCentreFinder.ino"
```

Expected footprint (with placeholder `Calibration.h`):

```
Sketch uses  20566 bytes (71%) of program storage space.
Global vars   1521 bytes (59%) of dynamic memory — 1039 bytes free.
```

There is no "Low memory available" warning — the large runtime
calibration buffer from the previous design is gone.

---

## 4. Open the Serial Monitor

In a second terminal:

```bash
arduino-cli monitor -p /dev/tty.usbmodem* --config baudrate=9600
```

On boot you should see:

```
 *** RADIAL GRADIENT CENTRE FINDER ***
Transfer function: compile-time (Calibration.h).
Place robot on radial gradient map, then press Button A.
```

Keep this terminal open for the entire session — it is the only source
of per-run quantitative output (positions, times, orient readings,
re-orient events).

---

## 5. Running a Centre-Finding Run

### 5.1 Acknowledge placement
Set the robot on the radial gradient map. Orientation does **not**
matter — the orient routine handles an arbitrary starting heading.
Avoid placing it exactly on the centre target.

OLED shows:
```
Place on
map. A?
```

Press **Button A**. A short 880 Hz beep confirms — the FSM advances to
`STATE_WAIT_START` and the OLED changes to `Ready / Press A`. (The
firmware zeroes the pose on this press so that the post-run `X/Y` is
reported relative to wherever the robot was sitting when you pressed.)

### 5.2 Start the run
Press **Button A** again. Serial prints:

```
--- RUN STARTED ---
```

The robot now executes the seven-phase FSM described in §8. You do not
interact further until the results screen appears.

### 5.3 What you should observe

| Phase | What the robot does | OLED | Typical serial line |
|---|---|---|---|
| Orient null-seek | Rotates slowly in place, faster when mis-aligned | `Orient` / `dE:<%>` | `ORIENT  L0:.. L4:.. dE:..` (~5 Hz) |
| Measure A | Stops briefly (150 ms settle), records centre | `MeasA` | `ORIENT  null A: theta=.. deg, centre_A=..%` |
| Flip 180° | Rotates exactly PI radians using encoder-integrated pose | `Flip180` | (silent) |
| Measure B | Stops briefly, records centre | `Check` / `A:.. B:..` | `ORIENT  null B: theta=.. deg, centre_B=..%` |
| Resolve | If A was darker, flips back another 180° | `Back180` | `ORIENT  chose heading A; flipping back` |
| Travel | Drives forward at `TRAVEL_SPEED`, monitors edges + centre | `Going` / `C:<%>` | `TRAVEL  C:.. dark:.. dE:..` (~5 Hz) |
| Re-orient (if triggered) | Stops, returns to orient null-seek | `Orient` | `TRAVEL  lost alignment — re-orienting` |
| Arrival | Stops, plays victory chord (1100/1320/1540 Hz) | `X:.. Y:..` / `T:..s` (held 20 s) | `=== RUN RESULTS === Reason: found` |

---

## 6. Reading the Results

When arrival or timeout triggers, the OLED shows:

```
X:<x_mm> Y:<y_mm>
T:<seconds>s
```

- `X`, `Y` — integer mm, from the pose the robot was sitting in when
  you first pressed **Button A** (*not* a global origin).
- `T` — elapsed seconds since the second A press, to 2 d.p.

The screen is held for **at least 20 s** (`RESULT_DISPLAY_MS`) before
Button A is accepted again. Use this window to photograph or transcribe
the numbers.

Serial prints the same data with one extra field:

```
=== RUN RESULTS ===
  Reason   : found        (or "timeout")
  Position : X=.. mm, Y=.. mm
  Time     : ..s
===================
```

Reason `timeout` means the 60 s safety net fired before arrival — the
pose shown is wherever the robot happened to be at that instant.

---

## 7. Restarting

After the results hold elapses the robot enters `STATE_DONE` — the OLED
keeps showing the final X/Y/T. Pressing **Button A** now:

1. Beeps once at 440 Hz.
2. Resets to the placement prompt (`Place on / map. A?`).

You can move the robot to a new pose on the map and start another run.
**No re-calibration is needed** — the transfer function is compile-time
and stays valid until the environment changes. If lighting changes or
you print a new map, regenerate `Calibration.h` (§2) and re-flash (§3).

---

## 8. FSM State Reference

```
                            Button A
     STATE_WAIT_PLACEMENT ───────────▶ STATE_WAIT_START
                                            │ Button A
                                            │ (pose := 0, timer := 0)
                                            ▼
                                   STATE_ORIENT_SEEK_NULL  ◀──────────┐
                                            │ |L[0]-L[4]| < TOL        │
                                            │ for NULL_DWELL_MS        │
                                            ▼                          │
                                   STATE_ORIENT_MEASURE_A              │
                                            │ settle, record centre_A  │
                                            │ setRotation(PI)          │
                                            ▼                          │
                                   STATE_ORIENT_FLIP_180               │
                                            │ checkRotation() done     │
                                            ▼                          │
                                   STATE_ORIENT_MEASURE_B              │
                                            │ settle, record centre_B  │
                                            │ pick darker heading      │
                                            ▼                          │
                                   STATE_ORIENT_RESOLVE                │
                                            │ if A chosen, setRot(PI)  │
                                            │ then checkRotation()     │
                                            ▼                          │
                                   STATE_TRAVEL ── edge_err > LOST ────┘
                                            │
                                            │ centre found  |  timeout
                                            ▼
                                   STATE_AT_CENTRE
                                            │ after RESULT_DISPLAY_MS
                                            ▼
                                   STATE_DONE ──(Button A)──▶ WAIT_PLACEMENT
```

Key invariants:
- `L[i] = gradientToPercent(i, line_sensors.readings[i])` — uses
  compile-time `CAL_WHITE/BLACK/RANGE`; no raw ADC comparisons anywhere
  in the FSM.
- `edge_diff = L[0] − L[4]` drives the orient rotation direction and
  magnitude.
- `centre_A < centre_B` ⇒ B is already darker, proceed from there.
  Otherwise flip back to A.

---

## 9. Tuning Parameters

All tuning constants live near the top of `GradientCentreFinder.ino`,
each marked `// HUMAN TUNE`. Defaults are first-pass estimates; verify
them on your specific map before trusting a result. Tune in the order
listed.

### 9.1 Establish baseline sensor behaviour (static, pre-tune)

Before changing any constant, confirm the transfer function gives
sensible readings with the robot held still:

1. Flash the firmware (§3). Open serial (§4).
2. Place the robot on the radial map pointing *approximately* at the
   centre. Watch the `Orient dE:` serial line.
3. Rotate the robot by hand ±45° off-axis and back. You should see:
   - `dE` within a few % of 0 on-axis (target: < 5 %).
   - `dE` greater than ~20 % at ±45°.

If not, either `Calibration.h` is stale (re-run §2) or the sensors are
physically misaligned.

### 9.2 Null-seeking constants

| Constant | Default | Tune by observing… |
|---|---|---|
| `EDGE_EQUAL_TOLERANCE` | `3.0 %` | Set to ≈ 2–3× the observed `dE` noise at an on-axis static pose. If the robot never triggers null, raise it. If it triggers at random headings, lower it. |
| `NULL_DWELL_MS` | `120 ms` | If the null fires momentarily as the robot sweeps through zero, raise this to force a longer in-tolerance dwell. |
| `ORIENT_TURN_GAIN` | `0.010` | Halve if the robot hunts (oscillates past the null). Double if it's glacial from a large starting error. |
| `ORIENT_TURN_SPEED_MIN` | `0.10` | Below this, motor dead-bands eat the rotation. Raise if the robot visibly stalls near the null. |
| `ORIENT_TURN_SPEED_MAX` | `0.35` | Cap to limit overshoot. Lower if the robot sweeps past the null repeatedly. |
| `ORIENT_SETTLE_MS` | `150 ms` | Raise if `centre_A` / `centre_B` values in serial look noisy run-to-run for the same pose. |

### 9.3 Travel constants

| Constant | Default | Tune by observing… |
|---|---|---|
| `TRAVEL_SPEED` | `0.25` | Conservative — the algorithm *reacts* to drift, so too fast leaves the tolerance window before re-orient can catch up. Raise cautiously. |
| `CENTRE_DARKNESS_THRESHOLD` | `80.0 %` | Derive from the darkness reading the centre sensor gives at the true centre (serial `dark:` field while the robot sits on the bullseye). Pick ~80 % of that peak — the robot rarely sits perfectly on the target. |
| `CENTRE_CONFIRM_COUNT` | `3` | Raise if sporadic false arrivals trigger en route. |
| `EDGE_LOST_THRESHOLD` | `12.0 %` | Must be ≥ 3–4× `EDGE_EQUAL_TOLERANCE` to avoid chatter. |
| `LOST_CONFIRM_COUNT` | `4` | Raise if transient noise triggers re-orient on a clean radius. |
| `GRADIENT_TIMEOUT_MS` | `60000` | Leave as-is unless the map is very large. |

### 9.4 Do **not** tune

- PID gains `Kp=48, Ki=0.5`, the `0.88` right-bias in `setDemandSpeeds`,
  motor dead-bands in `Motors.h`, or kinematics constants. These are
  tuned for this specific robot.
- The offline calibration procedure — 25 shades × 30 samples is baked
  into `LineSensorTest.ino` and `transfer_function.py`.

---

## 10. Acceptance Test Suite

| # | Test |
|---|---|
| 1 | `Calibration.h` contains non-placeholder values generated by `transfer_function.py` for the current environment |
| 2 | Sketch compiles with `arduino-cli compile --fqbn pololu-a-star:avr:a-star32U4` without errors |
| 3 | Boot → OLED shows `Place on / map. A?`, no calibration/drive motion |
| 4 | First Button A press on the map resets pose, OLED changes to `Ready / Press A`, no motor motion |
| 5 | Second Button A press starts a run — serial prints `--- RUN STARTED ---` |
| 6 | Orient routine terminates within one revolution + one flip (no endless rotation) |
| 7 | Correct heading chosen: lay the robot so `centre_A` is the darker — verify serial `chose heading A; flipping back` and the robot does flip back |
| 8 | Travel phase arrives at centre on a clean start; victory chord plays |
| 9 | Deliberate perturbation mid-travel (nudge ±30°) triggers re-orient at least once |
| 10 | OLED displays `X:…`, `Y:…`, `T:…` and holds for ≥ 20 s |
| 11 | 60 s timeout on a deliberately corrupted run produces `Reason: timeout` in serial, X/Y still shown |
| 12 | Button A after the hold restarts from placement prompt — no re-calibration occurs |

---

## 11. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Every sensor reads `0 %` or `100 %` at all times | `Calibration.h` still has placeholder values, or CAL_RANGE is wrong for the current lighting. Re-run §2 and re-flash. |
| `Orient dE:` saturates near ±100 % and never crosses zero | Sensors are out of the calibrated ADC range (likely because CAL_WHITE/BLACK don't match the map). Re-run §2 on the *same* surface and lighting as the map. |
| `gradientToPercent` returns `-1.0` in serial | Bad `sensor_idx` or `CAL_RANGE[k] ≤ 0` — check `Calibration.h` sanity. |
| Robot rotates endlessly in orient phase | `EDGE_EQUAL_TOLERANCE` too tight, or sensors `L[0]`/`L[4]` read systematically different. Widen the tolerance; check sensor alignment. |
| Robot picks the wrong heading | `centre_A` and `centre_B` in serial are similar (within a few %). Raise `NULL_DWELL_MS` and/or `ORIENT_SETTLE_MS`. |
| Robot arrives "off-centre" | `CENTRE_DARKNESS_THRESHOLD` too low — raise so arrival fires only near the true peak. |
| Spurious re-orients during travel | `EDGE_LOST_THRESHOLD` too tight; raise it. Also check for lighting gradients on the map that aren't part of the print. |
| Button A seems to do nothing | 200 ms `BUTTON_DEBOUNCE_MS`. Also: during the 20 s `RESULT_DISPLAY_MS` hold, A is deliberately ignored. |
| Upload fails with "device busy" | A serial monitor is holding the port. Close it, upload, reopen. |

---

## 12. Quick-Reference Command Cheat Sheet

```bash
# --- Once per environment: generate Calibration.h ---
cd "SEMTM0042_43/3Pi_CodeStub/robotics-c1-codebase"
arduino-cli compile --fqbn pololu-a-star:avr:a-star32U4 LineSensorTest/LineSensorTest.ino
arduino-cli upload  -p /dev/tty.usbmodem* --fqbn pololu-a-star:avr:a-star32U4 LineSensorTest/LineSensorTest.ino
python log_sensors.py --shades 25
#   (capture each patch black→white with Button A, then Ctrl+C)
python transfer_function.py
cp 3Pi_CodeStub/Calibration.h "../../final code/GradientCentreFinder/Calibration.h"

# --- Once per session: build & flash the centre finder ---
cd ../..   # back to repo root
arduino-cli compile --fqbn pololu-a-star:avr:a-star32U4 \
  "final code/GradientCentreFinder/GradientCentreFinder.ino"
arduino-cli upload -p /dev/tty.usbmodem* --fqbn pololu-a-star:avr:a-star32U4 \
  "final code/GradientCentreFinder/GradientCentreFinder.ino"

# --- Serial monitor ---
arduino-cli monitor -p /dev/tty.usbmodem* --config baudrate=9600

# --- Per run ---
#   a. Place robot on map → Button A (placement reset, pose zeroed)
#   b. Button A → FSM runs → results held on OLED for 20 s
#   c. Button A after hold → restart from placement prompt
```
