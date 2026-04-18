/*************************************************************
 *  GradientCalibration.h — transfer-function wrapper
 *
 *  Converts a raw line-sensor ADC reading into a lightness
 *  percentage using the compile-time calibration arrays in
 *  Calibration.h.  No runtime calibration is performed — the
 *  parameters are generated offline by transfer_function.py and
 *  flashed into the robot alongside the sketch.
 *
 *  Primary transfer function (per sensor k) — POWER-LAW:
 *
 *      pct = 100 * ( (CAL_A[k] - raw) / CAL_B[k] ) ^ (1 / CAL_N[k])
 *
 *  Result is clamped to [0, 100] where
 *      0 % = black (centre of radial map),
 *    100 % = white (perimeter of radial map).
 *
 *  The power-law form is the physically-correct transfer
 *  function for the 3Pi+ sensor stack (phototransistor + pull-up
 *  divider + printed-gamma reflectance).  A linear fit over 26
 *  shades gave R² ≈ 0.85 with pronounced U-shaped residuals; the
 *  three-parameter power fit gives R² ≈ 0.995 with no systematic
 *  residual — see TransferFunction_Analysis.md.
 *
 *  gradientToPercentLinear() is kept available for A/B comparison
 *  and falls back to the legacy CAL_WHITE/CAL_BLACK/CAL_RANGE.
 *
 *  Depends on: LineSensors.h  (NUM_SENSORS)
 *              Calibration.h  (CAL_A / CAL_B / CAL_N and legacy arrays)
 *************************************************************/

#ifndef _GRADIENT_CALIBRATION_H
#define _GRADIENT_CALIBRATION_H

#include <math.h>
#include "LineSensors.h"
#include "Calibration.h"


/*
 * Convert a single raw ADC reading to a lightness percentage
 * using the calibrated POWER-LAW transfer function.
 *
 * @param sensor_idx   Sensor index [0 .. NUM_SENSORS-1]
 * @param raw_reading  Raw analogRead() value [0 .. 1023]
 * @return             Lightness % [0.0 = black .. 100.0 = white]
 *                     Returns -1.0 on bad sensor_idx / bad params.
 */
float gradientToPercent(int sensor_idx, float raw_reading) {

  if (sensor_idx < 0 || sensor_idx >= NUM_SENSORS) return -1.0f;

  // Guard against bad calibration data (zero denominator, zero exponent)
  float b = CAL_B[sensor_idx];
  float n = CAL_N[sensor_idx];
  if (b <= 0.0f || n <= 0.0f) return -1.0f;

  // (CAL_A - raw) / CAL_B  is the forward fraction  (L/100)^n.
  // Clamp to [0, 1]: negative values appear when the sensor reads
  // slightly above CAL_A due to noise at the black end.
  float frac = (CAL_A[sensor_idx] - raw_reading) / b;
  if (frac <= 0.0f) return 0.0f;   // saturated at black
  if (frac >= 1.0f) return 100.0f; // saturated at white

  // Invert the power law to recover lightness %.
  float pct = 100.0f * powf(frac, 1.0f / n);

  // Final belt-and-braces clamp in case of pow domain edge cases.
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}


/*
 * Legacy linear transfer:  pct = 100 * (CAL_BLACK - raw) / CAL_RANGE.
 * Retained for diagnostics / A-B comparison against the power-law
 * form.  NOT called from the main centre-finder path.
 */
float gradientToPercentLinear(int sensor_idx, float raw_reading) {

  if (sensor_idx < 0 || sensor_idx >= NUM_SENSORS) return -1.0f;

  float range = CAL_RANGE[sensor_idx];
  if (range <= 0.0f) return -1.0f;

  float pct = 100.0f * (CAL_BLACK[sensor_idx] - raw_reading) / range;
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}


/*
 * Mean lightness across all five sensors (power-law path).
 * 0 % = black, 100 % = white.
 */
float getAverageLightness(float raw_readings[NUM_SENSORS]) {
  float sum = 0.0f;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sum += gradientToPercent(i, raw_readings[i]);
  }
  return sum / (float)NUM_SENSORS;
}


#endif  // _GRADIENT_CALIBRATION_H
