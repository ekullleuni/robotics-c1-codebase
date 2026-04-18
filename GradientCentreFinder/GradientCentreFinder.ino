/******************************************
  Radial-Gradient Centre-Finder FSM

  Null-seeking orientation + straight-line travel on a radial
  grayscale gradient map.  See RadialGradient_FSM_Plan.md.

  This sketch performs NO runtime sensor calibration.  The
  per-sensor transfer-function anchors are baked in at compile
  time via Calibration.h — generate them offline with the
  LineSensorTest workflow (see RadialGradient_CentreFinder_Workflow.md).

  Flow per run:
    1. STATE_WAIT_START          — power-on / post-run; Button A starts run (zeroes pose + timer).
    2. STATE_ORIENT_SEEK_NULL    — rotate in-place until L[0] == L[4].
    4. STATE_ORIENT_MEASURE_A    — record centre reading A.
    5. STATE_ORIENT_FLIP_180     — non-blocking 180° flip.
    6. STATE_ORIENT_MEASURE_B    — record centre reading B.
    7. STATE_ORIENT_RESOLVE      — if A was darker, flip back.
    8. STATE_TRAVEL              — drive forward, monitor for arrival
                                   or loss of alignment.
    9. STATE_AT_CENTRE / DONE    — display results, Button A restarts.

  SEMTM0042/43: University of Bristol.
*******************************************/

// ============================================
// INCLUDES
// ============================================
#include "Motors.h"
#include "PID.h"
#include "LineSensors.h"
#include "Kinematics.h"
#include "Encoders.h"
#include "GradientCalibration.h"

// OLED display (blue, single row of pins)
#include "oled.h"
OLED_c display(1, 30, 0, 17, 13);

// ============================================
// PIN DEFINITIONS
// ============================================
#define BUZZER_PIN   6
#define BUTTON_A_PIN 14

// ============================================
// HARDWARE INSTANCES
// ============================================
Motors_c      motors;
LineSensors_c line_sensors;
Kinematics_c  pose;
PID_c         left_pid;
PID_c         right_pid;

// ============================================
// SPEED MEASUREMENT
// ============================================
unsigned long speed_measurement_ts = 0;
#define SPEED_MEASUREMENT_INTERVAL 20   // ms

long  prev_count_e0 = 0;
long  prev_count_e1 = 0;
float speed_e0 = 0.0;   // RIGHT wheel (encoder 0)
float speed_e1 = 0.0;   // LEFT  wheel (encoder 1)

// ============================================
// PID CONTROL
// ============================================
unsigned long pid_update_ts = 0;
#define PID_UPDATE_INTERVAL 20          // ms

float demand_speed_left  = 0.0;
float demand_speed_right = 0.0;
bool  use_pid_control    = false;
float left_pwm_output    = 0.0;
float right_pwm_output   = 0.0;

// ============================================
// KINEMATICS & TURN/ROTATION
// ============================================
unsigned long pose_update_ts = 0;
#define POSE_UPDATE_INTERVAL 10         // ms

// --- Absolute turn (setTurn / checkTurn) ---
float target_angle = 0.0;
bool  turning      = false;
#define TURN_THRESHOLD  0.01
#define TURN_GAIN       5.0
#define MAX_TURN_SPEED  0.5
#define MIN_TURN_SPEED  0.15

// --- Relative rotation (setRotation / checkRotation) ---
float rotate_target      = 0.0;
float rotate_accumulated = 0.0;
float rotate_prev_theta  = 0.0;
bool  rotating           = false;

// ============================================
// RADIAL GRADIENT FSM
// ============================================
enum GradientState {
  STATE_WAIT_START,         // Power-on / post-run; A places robot and starts run
  STATE_ORIENT_SEEK_NULL,   // Rotating; looking for edge sensors to equalise
  STATE_ORIENT_MEASURE_A,   // Null found; settle, then record centre reading A
  STATE_ORIENT_FLIP_180,    // Non-blocking 180° rotation
  STATE_ORIENT_MEASURE_B,   // After flip; settle, then record centre B and choose darker
  STATE_ORIENT_RESOLVE,     // If chosen heading was A, rotate 180° back; else proceed
  STATE_TRAVEL,             // Drive forward; monitor for arrival or lost alignment
  STATE_AT_CENTRE,          // Centre found (or timeout); hold results on OLED
  STATE_DONE                // Idle; Button A restarts
};
GradientState grad_state = STATE_WAIT_START;

// --- Run timer ---
unsigned long run_start_ms = 0;

// --- Null-seeking ---
#define ORIENT_TURN_SPEED_MIN  0.10f    // counts/ms — HUMAN TUNE
#define ORIENT_TURN_SPEED_MAX  0.6f    // counts/ms — HUMAN TUNE
#define ORIENT_TURN_GAIN       0.015f   // counts/ms per % — HUMAN TUNE
#define EDGE_EQUAL_TOLERANCE   3.0f     // % — HUMAN TUNE
#define NULL_DWELL_MS          120UL    // ms — HUMAN TUNE
#define ORIENT_SETTLE_MS       150UL    // ms — HUMAN TUNE

unsigned long null_dwell_start_ts = 0;
unsigned long settle_start_ts     = 0;
float theta_A   = 0.0f;
float theta_B   = 0.0f;
float centre_A  = 0.0f;
float centre_B  = 0.0f;
float chosen_theta = 0.0f;

// --- Travel ---
#define TRAVEL_SPEED                   0.25f   // counts/ms — HUMAN TUNE
#define EDGE_LOST_THRESHOLD            12.0f   // % — HUMAN TUNE
#define LOST_CONFIRM_COUNT             4       // consecutive readings
#define CENTRE_DARKNESS_THRESHOLD      99.0f   // % (100 - L[2]) — HUMAN TUNE
#define CENTRE_CONFIRM_COUNT           3       // consecutive readings
#define GRADIENT_TIMEOUT_MS            60000UL // 60 s safety net
#define TRAVEL_LOOP_READ_INTERVAL_MS   20UL    // ADC throttle

int arrival_confirm = 0;
int lost_confirm    = 0;
unsigned long travel_read_ts = 0;

// --- Results capture ---
float         captured_x = 0.0f;
float         captured_y = 0.0f;
unsigned long captured_time_ms = 0;
const char*   captured_reason  = "";

// --- Display ---
unsigned long display_update_ts = 0;
#define DISPLAY_UPDATE_INTERVAL_MS 250UL
unsigned long display_hold_start_ts = 0;
#define RESULT_DISPLAY_MS          20000UL
bool results_painted = false;

// --- Serial debug throttle ---
unsigned long debug_print_ts = 0;
#define DEBUG_PRINT_INTERVAL 200

// --- Extra debug timestamps ---
unsigned long btn_debug_ts       = 0;
unsigned long placement_debug_ts = 0;

// --- Button debounce ---
bool  button_last_state    = HIGH;
unsigned long button_debounce_ts = 0;
#define BUTTON_DEBOUNCE_MS 200


// ************************************************************
//                         setup()
// ************************************************************
void setup() {

  // --- Hardware init ---
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  motors.initialise();
  line_sensors.initialiseForADC();
  setupEncoder0();
  setupEncoder1();
  pose.initialise(0, 0, 0);

  Serial.begin(9600);
  delay(500);
  Serial.println(" *** RADIAL GRADIENT CENTRE FINDER *** ");
  Serial.println("Transfer function: compile-time (Calibration.h).");

  // --- PID init (gains tuned for this specific robot — do not change) ---
  float Kp = 48.0, Ki = 0.5, Kd = 0.0;
  left_pid.initialise(Kp, Ki, Kd);
  right_pid.initialise(Kp, Ki, Kd);
  use_pid_control = true;
  pid_update_ts = millis();

  // --- Speed measurement init ---
  prev_count_e0 = count_e0;
  prev_count_e1 = count_e1;
  speed_measurement_ts = millis();

  // --- OLED: initial prompt ---
  display.reset();
  display.clear();
  display.gotoXY(0, 0);
  display.print("Place on");
  display.gotoXY(0, 1);
  display.print("map. A!");

  pose_update_ts = millis();
  grad_state = STATE_WAIT_START;

  // tone(BUZZER_PIN, 440, 100);
  Serial.println("Place robot on radial gradient map, then press Button A to start.");
  Serial.print("DBG: BUTTON_A_PIN="); Serial.print(BUTTON_A_PIN);
  Serial.print(" initial read="); Serial.println(digitalRead(BUTTON_A_PIN));
}


// ************************************************************
//  NON-BLOCKING BUTTON CHECK (rising-edge with debounce)
// ************************************************************
bool buttonAPressed() {
  bool raw = digitalRead(BUTTON_A_PIN);
  unsigned long now = millis();

  // Print pin state every 1 s so we can see it in the serial monitor
  if (now - btn_debug_ts >= 1000) {
    Serial.print("DBG btn: raw="); Serial.print(raw);
    Serial.print(" last="); Serial.print(button_last_state);
    Serial.print(" debounce_age="); Serial.println(now - button_debounce_ts);
    btn_debug_ts = now;
  }

  if (now - button_debounce_ts < BUTTON_DEBOUNCE_MS) {
    button_last_state = raw;
    return false;
  }

  if (button_last_state == HIGH && raw == LOW) {
    Serial.println("DBG btn: PRESS DETECTED");
    button_debounce_ts = now;
    button_last_state = raw;
    return true;
  }

  button_last_state = raw;
  return false;
}


// ************************************************************
//  SYSTEM UPDATES (called every loop iteration)
// ************************************************************

void measureSpeeds() {
  unsigned long now = millis();
  unsigned long dt  = now - speed_measurement_ts;

  if (dt >= SPEED_MEASUREMENT_INTERVAL) {
    long local_e0, local_e1;
    noInterrupts();
    local_e0 = count_e0;
    local_e1 = count_e1;
    interrupts();

    speed_e0 = (float)(local_e0 - prev_count_e0) / (float)dt;
    speed_e1 = (float)(local_e1 - prev_count_e1) / (float)dt;

    prev_count_e0 = local_e0;
    prev_count_e1 = local_e1;
    speed_measurement_ts = now;
  }
}

void updatePID() {
  unsigned long now = millis();
  if (now - pid_update_ts < PID_UPDATE_INTERVAL) return;

  if (use_pid_control) {
    if (demand_speed_left == 0.0) {
      left_pwm_output = 0.0;
      left_pid.reset();
    } else {
      left_pwm_output = left_pid.update(demand_speed_left, speed_e1);
    }
    if (demand_speed_right == 0.0) {
      right_pwm_output = 0.0;
      right_pid.reset();
    } else {
      right_pwm_output = right_pid.update(demand_speed_right, speed_e0);
    }
    motors.setPWM(left_pwm_output, right_pwm_output);
  } else {
    left_pid.reset();
    right_pid.reset();
  }

  pid_update_ts = now;
}

void updatePose() {
  unsigned long now = millis();
  if (now - pose_update_ts >= POSE_UPDATE_INTERVAL) {
    pose.update();
    pose_update_ts = now;
  }
}


// ************************************************************
//  MOTOR DEMAND HELPERS
// ************************************************************
void setDemandSpeeds(float left_speed, float right_speed) {
  demand_speed_left  = left_speed;
  demand_speed_right = 0.88 * right_speed;   // hardware bias correction
}

void setDemandSpeed(float speed) {
  setDemandSpeeds(speed, speed);
}

void stopWithPID() {
  setDemandSpeeds(0.0, 0.0);
  motors.setPWM(0, 0);
  left_pid.reset();
  right_pid.reset();
}


// ************************************************************
//  NAVIGATION HELPERS
// ************************************************************
float angleDiff(float target, float current) {
  float diff = target - current;
  return atan2(sin(diff), cos(diff));
}


// ============================================
// ABSOLUTE TURN
// ============================================
void setTurn(float angle) {
  target_angle = angle;
  turning = true;
}

bool checkTurn() {
  if (!turning) return true;

  float angle_error = angleDiff(target_angle, pose.theta);

  if (fabs(angle_error) < TURN_THRESHOLD) {
    stopWithPID();
    turning = false;
    return true;
  }

  float turn_speed = angle_error * TURN_GAIN;
  if (turn_speed >  MAX_TURN_SPEED) turn_speed =  MAX_TURN_SPEED;
  if (turn_speed < -MAX_TURN_SPEED) turn_speed = -MAX_TURN_SPEED;
  if (fabs(turn_speed) < MIN_TURN_SPEED) {
    turn_speed = (angle_error > 0) ? MIN_TURN_SPEED : -MIN_TURN_SPEED;
  }

  setDemandSpeeds(-turn_speed, turn_speed);
  return false;
}


// ============================================
// RELATIVE ROTATION
// ============================================
void setRotation(float radians) {
  rotate_target      = radians;
  rotate_accumulated = 0.0;
  rotate_prev_theta  = pose.theta;
  rotating           = true;
}

bool checkRotation() {
  if (!rotating) return true;

  float delta = angleDiff(pose.theta, rotate_prev_theta);
  rotate_accumulated += delta;
  rotate_prev_theta   = pose.theta;

  float remaining = rotate_target - rotate_accumulated;

  if (fabs(remaining) < TURN_THRESHOLD) {
    stopWithPID();
    rotating = false;
    return true;
  }

  float turn_speed = remaining * TURN_GAIN;
  if (turn_speed >  MAX_TURN_SPEED) turn_speed =  MAX_TURN_SPEED;
  if (turn_speed < -MAX_TURN_SPEED) turn_speed = -MAX_TURN_SPEED;
  if (fabs(turn_speed) < MIN_TURN_SPEED) {
    turn_speed = (remaining > 0) ? MIN_TURN_SPEED : -MIN_TURN_SPEED;
  }

  setDemandSpeeds(-turn_speed, turn_speed);
  return false;
}


// ************************************************************
//  FSM HELPERS
// ************************************************************

// Read all 5 sensors and fill L[] with transfer-function %s.
// L[0]=DN1 (left edge), L[2]=DN3 (centre), L[4]=DN5 (right edge).
// 0 % = black, 100 % = white.
void readLightness(float L[NUM_SENSORS]) {
  line_sensors.readSensorsADC();
  for (int i = 0; i < NUM_SENSORS; i++) {
    L[i] = gradientToPercent(i, line_sensors.readings[i]);
  }
}

// Paint the results screen once.  Left in place until the next
// run starts; updateGradientDisplay() skips redraw in the
// results-holding states so the numbers don't flicker.
void paintResultsScreen() {
  float elapsed_s = (float)captured_time_ms / 1000.0f;

  display.clear();
  display.gotoXY(0, 0);
  display.print("X:");
  display.print((int)captured_x);
  display.print(" Y:");
  display.print((int)captured_y);

  display.gotoXY(0, 1);
  display.print("T:");
  display.print(elapsed_s, 2);
  display.print("s");
}

void reportSerialResults() {
  float elapsed_s = (float)captured_time_ms / 1000.0f;
  Serial.println();
  Serial.println("=== RUN RESULTS ===");
  Serial.print("  Reason   : ");
  Serial.println(captured_reason);
  Serial.print("  Position : X=");
  Serial.print(captured_x, 1);
  Serial.print(" mm, Y=");
  Serial.print(captured_y, 1);
  Serial.println(" mm");
  Serial.print("  Time     : ");
  Serial.print(elapsed_s, 2);
  Serial.println(" s");
  Serial.println("===================");
}


// Non-blocking OLED update.  Skipped while results are being held.
void updateGradientDisplay() {
  unsigned long now = millis();
  if (now - display_update_ts < DISPLAY_UPDATE_INTERVAL_MS) return;
  display_update_ts = now;

  // Don't overwrite the results screen
  if (grad_state == STATE_AT_CENTRE || grad_state == STATE_DONE) return;

  display.clear();
  display.gotoXY(0, 0);

  switch (grad_state) {

    case STATE_WAIT_START:
      display.print("Place on");
      display.gotoXY(0, 1);
      display.print("map. A!");
      break;

    case STATE_ORIENT_SEEK_NULL: {
      float L[NUM_SENSORS];
      readLightness(L);
      float edge_err = fabs(L[0] - L[4]);
      display.print("Orient");
      display.gotoXY(0, 1);
      display.print("dE:");
      display.print(edge_err, 1);
      display.print("%");
      break;
    }

    case STATE_ORIENT_MEASURE_A:
      display.print("MeasA");
      break;

    case STATE_ORIENT_FLIP_180:
      display.print("Flip180");
      break;

    case STATE_ORIENT_MEASURE_B:
      display.print("Check");
      display.gotoXY(0, 1);
      display.print("A:");
      display.print(centre_A, 0);
      display.print(" B:");
      display.print(centre_B, 0);
      break;

    case STATE_ORIENT_RESOLVE:
      display.print("Back180");
      break;

    case STATE_TRAVEL: {
      float L[NUM_SENSORS];
      readLightness(L);
      display.print("Going");
      display.gotoXY(0, 1);
      display.print("C:");
      display.print(L[2], 1);
      display.print("%");
      break;
    }

    default:
      break;
  }
}


// ************************************************************
//                          loop()
// ************************************************************
void loop() {

  // --- Continuous system updates ---
  measureSpeeds();
  updatePID();
  updatePose();

  // --- FSM ---
  switch (grad_state) {

    // ========================================================
    // WAIT_START: place robot on map, press A once to start run
    // ========================================================
    case STATE_WAIT_START:
      if (millis() - placement_debug_ts >= 2000) {
        Serial.println("DBG state: WAIT_START (place robot on map, press A to start)");
        placement_debug_ts = millis();
      }
      if (buttonAPressed()) {
        Serial.println("DBG: A pressed in WAIT_START -> ORIENT_SEEK_NULL");
        pose.initialise(0, 0, 0);
        pose_update_ts      = millis();
        run_start_ms        = millis();
        null_dwell_start_ts = 0;
        arrival_confirm     = 0;
        lost_confirm        = 0;
        results_painted     = false;

        // tone(BUZZER_PIN, 880, 100);
        Serial.println();
        Serial.println("--- RUN STARTED ---");

        grad_state = STATE_ORIENT_SEEK_NULL;
      }
      break;

    // ========================================================
    // ORIENT_SEEK_NULL: rotate in-place until L[0] ~= L[4]
    // ========================================================
    case STATE_ORIENT_SEEK_NULL: {
      float L[NUM_SENSORS];
      readLightness(L);
      float edge_diff = L[0] - L[4];          // signed, %
      float edge_err  = fabs(edge_diff);

      // Proportional rotation speed.
      // edge_diff > 0 → left lighter → centre to RIGHT → rotate CW
      //   → turn_speed negative → setDemandSpeeds(+|t|, -|t|)
      // edge_diff < 0 → right lighter → centre to LEFT → rotate CCW
      //   → turn_speed positive → setDemandSpeeds(-|t|, +|t|)
      float turn_speed = -ORIENT_TURN_GAIN * edge_diff;
      float mag = fabs(turn_speed);
      if (mag < ORIENT_TURN_SPEED_MIN) mag = ORIENT_TURN_SPEED_MIN;
      if (mag > ORIENT_TURN_SPEED_MAX) mag = ORIENT_TURN_SPEED_MAX;
      turn_speed = (turn_speed >= 0) ? mag : -mag;
      setDemandSpeeds(-turn_speed, turn_speed);

      // Throttled debug trace
      if (millis() - debug_print_ts >= DEBUG_PRINT_INTERVAL) {
        Serial.print("ORIENT  L0:");
        Serial.print(L[0], 1);
        Serial.print(" L4:");
        Serial.print(L[4], 1);
        Serial.print(" dE:");
        Serial.println(edge_diff, 2);
        debug_print_ts = millis();
      }

      // Dwell check: edges stay close for NULL_DWELL_MS
      if (edge_err < EDGE_EQUAL_TOLERANCE) {
        if (null_dwell_start_ts == 0) null_dwell_start_ts = millis();
        if (millis() - null_dwell_start_ts >= NULL_DWELL_MS) {
          stopWithPID();
          settle_start_ts = millis();
          grad_state = STATE_ORIENT_MEASURE_A;
        }
      } else {
        null_dwell_start_ts = 0;
      }
      break;
    }

    // ========================================================
    // ORIENT_MEASURE_A: settle, sample centre, command 180° flip
    // ========================================================
    case STATE_ORIENT_MEASURE_A: {
      if (millis() - settle_start_ts < ORIENT_SETTLE_MS) break;

      float L[NUM_SENSORS];
      readLightness(L);
      theta_A  = pose.theta;
      centre_A = L[2];

      Serial.print("ORIENT  null A: theta=");
      Serial.print(theta_A * 180.0f / PI, 1);
      Serial.print(" deg, centre_A=");
      Serial.print(centre_A, 1);
      Serial.println("%");

      setRotation(PI);
      grad_state = STATE_ORIENT_FLIP_180;
      break;
    }

    // ========================================================
    // ORIENT_FLIP_180: non-blocking 180° rotation
    // ========================================================
    case STATE_ORIENT_FLIP_180:
      if (checkRotation()) {
        stopWithPID();
        settle_start_ts = millis();
        grad_state = STATE_ORIENT_MEASURE_B;
      }
      break;

    // ========================================================
    // ORIENT_MEASURE_B: settle, sample centre, pick darker heading
    // ========================================================
    case STATE_ORIENT_MEASURE_B: {
      if (millis() - settle_start_ts < ORIENT_SETTLE_MS) break;

      float L[NUM_SENSORS];
      readLightness(L);
      theta_B  = pose.theta;
      centre_B = L[2];

      Serial.print("ORIENT  null B: theta=");
      Serial.print(theta_B * 180.0f / PI, 1);
      Serial.print(" deg, centre_B=");
      Serial.print(centre_B, 1);
      Serial.println("%");

      if (centre_B < centre_A) {
        // Already facing darker direction — no further turn.
        chosen_theta = theta_B;
        Serial.println("ORIENT  chose heading B");
        travel_read_ts = millis();
        arrival_confirm = 0;
        lost_confirm    = 0;
        grad_state = STATE_TRAVEL;
      } else {
        chosen_theta = theta_A;
        Serial.println("ORIENT  chose heading A; flipping back");
        setRotation(PI);
        grad_state = STATE_ORIENT_RESOLVE;
      }
      break;
    }

    // ========================================================
    // ORIENT_RESOLVE: second 180° flip back to theta_A
    // ========================================================
    case STATE_ORIENT_RESOLVE:
      if (checkRotation()) {
        stopWithPID();
        travel_read_ts  = millis();
        arrival_confirm = 0;
        lost_confirm    = 0;
        grad_state = STATE_TRAVEL;
      }
      break;

    // ========================================================
    // TRAVEL: straight forward, monitor for arrival or drift
    // ========================================================
    case STATE_TRAVEL: {
      setDemandSpeed(TRAVEL_SPEED);

      // Safety timeout always active
      if (millis() - run_start_ms >= GRADIENT_TIMEOUT_MS) {
        stopWithPID();
        captured_x        = pose.x;
        captured_y        = pose.y;
        captured_time_ms  = millis() - run_start_ms;
        captured_reason   = "timeout";
        Serial.println("*** TIMEOUT ***");
        reportSerialResults();
        display_hold_start_ts = millis();
        grad_state = STATE_AT_CENTRE;
        break;
      }

      // Throttle ADC reads
      if (millis() - travel_read_ts < TRAVEL_LOOP_READ_INTERVAL_MS) break;
      travel_read_ts = millis();

      float L[NUM_SENSORS];
      readLightness(L);
      float edge_err        = fabs(L[0] - L[4]);
      float darkness_centre = 100.0f - L[2];

      // Throttled debug trace
      if (millis() - debug_print_ts >= DEBUG_PRINT_INTERVAL) {
        Serial.print("TRAVEL  C:");
        Serial.print(L[2], 1);
        Serial.print(" dark:");
        Serial.print(darkness_centre, 1);
        Serial.print(" dE:");
        Serial.println(edge_err, 2);
        debug_print_ts = millis();
      }

      // Arrival
      if (darkness_centre > CENTRE_DARKNESS_THRESHOLD) {
        arrival_confirm++;
      } else {
        arrival_confirm = 0;
      }

      if (arrival_confirm >= CENTRE_CONFIRM_COUNT) {
        stopWithPID();
        captured_x       = pose.x;
        captured_y       = pose.y;
        captured_time_ms = millis() - run_start_ms;
        captured_reason  = "found";

        // tone(BUZZER_PIN, 1100, 150); delay(200);
        // tone(BUZZER_PIN, 1320, 150); delay(200);
        // tone(BUZZER_PIN, 1540, 300);

        reportSerialResults();
        display_hold_start_ts = millis();
        grad_state = STATE_AT_CENTRE;
        break;
      }

      // Lost alignment
      if (edge_err > EDGE_LOST_THRESHOLD) {
        lost_confirm++;
      } else {
        lost_confirm = 0;
      }

      if (lost_confirm >= LOST_CONFIRM_COUNT) {
        stopWithPID();
        Serial.println("TRAVEL  lost alignment — re-orienting");
        null_dwell_start_ts = 0;
        grad_state = STATE_ORIENT_SEEK_NULL;
        break;
      }
      break;
    }

    // ========================================================
    // AT_CENTRE: hold results on OLED for RESULT_DISPLAY_MS
    // ========================================================
    case STATE_AT_CENTRE:
      if (!results_painted) {
        paintResultsScreen();
        results_painted = true;
      }
      if (millis() - display_hold_start_ts >= RESULT_DISPLAY_MS) {
        grad_state = STATE_DONE;
      }
      // Button A deliberately ignored during the hold.
      break;

    // ========================================================
    // DONE: Button A restarts — place robot and press A again
    // ========================================================
    case STATE_DONE:
      if (buttonAPressed()) {
        // tone(BUZZER_PIN, 440, 100);
        Serial.println();
        Serial.println("--- RESTARTING ---");
        Serial.println("Reposition robot, then press Button A to start.");
        results_painted = false;
        grad_state = STATE_WAIT_START;
      }
      break;

  } // end switch

  updateGradientDisplay();
}
