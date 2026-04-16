/******************************************
 *  LINE SENSOR SCIENTIFIC ANALYSIS SKETCH
 *
 *  Dedicated test firmware for characterising
 *  the 3Pi+ line sensor response.  Replaces
 *  the foraging FSM with three test phases:
 *
 *  CALIBRATION (starts immediately on power-on):
 *      Human places robot on each shade patch,
 *      from BLACK first to WHITE last.
 *      Press A to record the current shade.
 *      Press B when all shades have been recorded.
 *      Any number of shades are supported — the
 *      robot assumes equal spacing between black
 *      and white and sets line_sensors min/max/
 *      scaling for all subsequent calibrated reads.
 *
 *  Phase 2 (GRADIENT): Continuous logging while
 *      the robot is pushed along a gradient map.
 *      Press B to start/stop logging for each
 *      gradient strip.  Encoder counts give
 *      position along the gradient.
 *
 *  Phase 3 (THINLINE): Robot drives across a
 *      thin black line at multiple angles,
 *      logging sensors + encoders.  Autonomous
 *      drive, reverse, turn cycle.
 *
 *  Button A: record shade (calibration) / advance phase
 *  Button B: finalise calibration / trigger within phase
 *
 *  Serial: 115200 baud, CSV output
 ******************************************/

// ============================================
// INCLUDES
// ============================================
#include "Motors.h"
#include "PID.h"
#include "LineSensors.h"
#include "Encoders.h"
#include "Kinematics.h"
#include "SpeedMeasurement.h"  // countsToMM() for encoder → distance conversion

// ============================================
// PIN DEFINITIONS
// ============================================
#define BUZZER_PIN    6
#define BUTTON_A_PIN  14   // Active-low with internal pull-up
#define BUTTON_B_PIN  30   // Active-low (shared with TX LED on 3Pi+ 32U4)

// ============================================
// TEST CONFIGURATION
// ============================================

// Calibration phase
#define MAX_CALIB_SHADES       50   // Hard upper limit on recorded shades (safety cap)
#define CALIB_SAMPLES_PER_SHADE 20  // ADC readings averaged per shade
#define SAMPLE_DELAY_MS          5  // Delay between consecutive ADC reads (let ADC settle)

// Phase 2: Gradient test
#define GRADIENT_LOG_INTERVAL_MS 10  // Log every 10 ms while being pushed

// Phase 3: Thin-line test
#define THINLINE_DRIVE_DIST_MM 150.0  // Drive distance per crossing (mm)
#define THINLINE_DRIVE_SPEED   0.2    // Slow forward speed (counts/ms)
#define THINLINE_LOG_INTERVAL_MS  5   // Log every 5 ms during crossing
#define NUM_THINLINE_ANGLES     7     // Number of crossing angles
// Angles in degrees relative to perpendicular (0 = head-on)
const int thinline_angles[NUM_THINLINE_ANGLES] = { 0, 15, 30, 45, 60, 75, 90 };

// ============================================
// STATE MACHINE
// ============================================
enum TestPhase {
  // ---- Calibration (robot starts here) ----
  TEST_CALIB_WAITING,         // Waiting for A (record shade) or B (done)
  TEST_CALIB_SAMPLING,        // Actively averaging samples for the current shade

  // ---- Phase 2: Gradient test ----
  TEST_GRADIENT_READY,        // Waiting for B to start a gradient strip
  TEST_GRADIENT_LOGGING,      // Actively logging sensor data while pushed

  // ---- Phase 3: Thin-line test ----
  TEST_THINLINE_READY,        // Waiting for B to start the drive sequence
  TEST_THINLINE_DRIVE,        // Driving across the line
  TEST_THINLINE_REVERSE,      // Reversing back to start position
  TEST_THINLINE_TURN,         // Turning to next crossing angle

  // ---- Done ----
  TEST_COMPLETE
};

// Robot starts directly in calibration
TestPhase test_phase = TEST_CALIB_WAITING;

// ============================================
// HARDWARE INSTANCES
// ============================================
Motors_c      motors;
LineSensors_c line_sensors;
Kinematics_c  pose;
PID_c         left_pid;
PID_c         right_pid;

// ============================================
// SPEED MEASUREMENT (from Labsheet 1/3)
// ============================================
unsigned long speed_ts = 0;
#define SPEED_INTERVAL 20       // ms between speed calculations
long prev_e0 = 0, prev_e1 = 0; // previous encoder counts
float speed_e0 = 0.0;          // RIGHT wheel speed (counts/ms)
float speed_e1 = 0.0;          // LEFT  wheel speed (counts/ms)

// ============================================
// PID CONTROL
// ============================================
unsigned long pid_ts = 0;
#define PID_INTERVAL 20
float demand_left  = 0.0;
float demand_right = 0.0;
bool  pid_on = false;

// ============================================
// POSE UPDATE
// ============================================
unsigned long pose_ts = 0;
#define POSE_INTERVAL 10

// ============================================
// BUTTON DEBOUNCE
// ============================================
// Simple edge-detect: triggers once per press
// Returns true on the falling edge (HIGH → LOW)
unsigned long btn_a_last = 0;
unsigned long btn_b_last = 0;
bool btn_a_prev = true;  // HIGH = not pressed (pull-up)
bool btn_b_prev = true;
#define DEBOUNCE_MS 200

bool buttonPressed(int pin, bool &prev_state, unsigned long &last_time) {
  bool current = digitalRead(pin);
  if (!current && prev_state && (millis() - last_time >= DEBOUNCE_MS)) {
    // Falling edge detected (button just pressed)
    last_time = millis();
    prev_state = current;
    return true;
  }
  prev_state = current;
  return false;
}

// Shorthand
bool buttonA() { return buttonPressed(BUTTON_A_PIN, btn_a_prev, btn_a_last); }
bool buttonB() { return buttonPressed(BUTTON_B_PIN, btn_b_prev, btn_b_last); }

// ============================================
// CALIBRATION VARIABLES
// ============================================
// Stores the averaged ADC readings for each shade captured.
// Index 0 = black (first pressed), last index = white (last pressed).
float calib_readings[MAX_CALIB_SHADES][NUM_SENSORS];
int   calib_shade_count   = 0;   // Number of shades recorded so far
int   calib_sample_count  = 0;   // Samples accumulated for the shade being recorded
float calib_accum[NUM_SENSORS];  // Running sum for the current shade's samples

// ============================================
// PHASE 2 (GRADIENT) VARIABLES
// ============================================
int gradient_id = 0;      // Current gradient strip (incremented on each B press)
unsigned long gradient_log_ts = 0;

// ============================================
// PHASE 3 (THINLINE) VARIABLES
// ============================================
int   tl_angle_idx = 0;   // Index into thinline_angles[]
float tl_start_x = 0;     // Start position for distance tracking
float tl_start_y = 0;
unsigned long tl_log_ts = 0;
int   tl_pass_id = 0;     // Pass counter for CSV output

// Navigation helpers reused from main codebase
float target_angle_nav = 0.0;
bool  nav_turning = false;
bool  nav_reversing = false;
float rev_start_x = 0, rev_start_y = 0;
float rev_target = 0;

// ============================================
// NAVIGATION HELPER FUNCTIONS
// (Minimal subset from the main codebase)
// ============================================

// Smallest angular difference in [-PI, PI]
float angleDiff(float a, float b) {
  float d = a - b;
  return atan2(sin(d), cos(d));
}

// Distance from pose to point
float distTo(float tx, float ty) {
  float dx = tx - pose.x;
  float dy = ty - pose.y;
  return sqrt(dx * dx + dy * dy);
}

// Set PID demand speeds
void setDemand(float left, float right) {
  demand_left  = left;
  demand_right = 0.88 * right;  // Right motor calibration factor (from main codebase)
}

// Stop motors and reset PIDs
void stopMotors() {
  setDemand(0, 0);
  motors.setPWM(0, 0);
  left_pid.reset();
  right_pid.reset();
}

// Start a relative rotation (radians, positive = CCW)
float rot_target = 0, rot_accum = 0, rot_prev_th = 0;
bool  rot_active = false;

void startRotation(float radians) {
  rot_target  = radians;
  rot_accum   = 0;
  rot_prev_th = pose.theta;
  rot_active  = true;
}

// Returns true when rotation is complete
bool updateRotation() {
  if (!rot_active) return true;
  float delta = angleDiff(pose.theta, rot_prev_th);
  rot_accum  += delta;
  rot_prev_th = pose.theta;
  float remaining = rot_target - rot_accum;
  if (fabs(remaining) < 0.02) {  // ~1° threshold
    stopMotors();
    rot_active = false;
    return true;
  }
  // Proportional speed control
  float spd = remaining * 5.0;   // TURN_GAIN
  if (spd >  0.5) spd =  0.5;   // MAX_TURN_SPEED
  if (spd < -0.5) spd = -0.5;
  if (fabs(spd) < 0.15) spd = (remaining > 0) ? 0.15 : -0.15;
  setDemand(-spd, spd);
  return false;
}

// Start a reverse by distance (mm)
void startReverse(float dist_mm) {
  rev_start_x = pose.x;
  rev_start_y = pose.y;
  rev_target  = dist_mm;
  nav_reversing = true;
}

// Returns true when reverse distance is complete
bool updateReverse() {
  if (!nav_reversing) return true;
  float dx = pose.x - rev_start_x;
  float dy = pose.y - rev_start_y;
  if (sqrt(dx * dx + dy * dy) >= rev_target) {
    stopMotors();
    nav_reversing = false;
    return true;
  }
  setDemand(-0.2, -0.2);  // Slow reverse
  return false;
}

// ============================================
// SYSTEM UPDATE FUNCTIONS
// ============================================

// Measure wheel speeds from encoder deltas
void measureSpeeds() {
  unsigned long now = millis();
  if (now - speed_ts >= SPEED_INTERVAL) {
    long e0, e1;
    noInterrupts();
    e0 = count_e0;
    e1 = count_e1;
    interrupts();
    float dt = (float)(now - speed_ts);
    speed_e0 = (float)(e0 - prev_e0) / dt;
    speed_e1 = (float)(e1 - prev_e1) / dt;
    prev_e0 = e0;
    prev_e1 = e1;
    speed_ts = now;
  }
}

// Update PID and apply motor commands
void updatePID() {
  if (millis() - pid_ts < PID_INTERVAL) return;
  pid_ts = millis();
  if (!pid_on) {
    left_pid.reset();
    right_pid.reset();
    return;
  }
  float l_pwm, r_pwm;
  // e1 = LEFT, e0 = RIGHT (from Encoders.h mapping)
  if (demand_left == 0.0) { l_pwm = 0; left_pid.reset(); }
  else l_pwm = left_pid.update(demand_left, speed_e1);
  if (demand_right == 0.0) { r_pwm = 0; right_pid.reset(); }
  else r_pwm = right_pid.update(demand_right, speed_e0);
  motors.setPWM(l_pwm, r_pwm);
}

// Update pose from encoders
void updatePose() {
  if (millis() - pose_ts >= POSE_INTERVAL) {
    pose.update();
    pose_ts = millis();
  }
}

// ============================================
// CSV OUTPUT HELPERS
// ============================================

// Print raw ADC readings for all 5 sensors as CSV fields
// Format: s0, s1, s2, s3, s4
void printSensorCSV() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(line_sensors.readings[i], 0);
    if (i < NUM_SENSORS - 1) Serial.print(F(","));
  }
}

// ============================================
// CALIBRATION FUNCTION
// ============================================
// Called when the human presses B to signal that all shades
// have been recorded.  Uses the first recorded shade as BLACK
// and the last as WHITE, setting line_sensors.minimum[],
// maximum[], and scaling[] for use by calcCalibratedADC().
//
// On the 3Pi+ hardware with INPUT_PULLUP, a black surface
// gives HIGHER raw ADC values (phototransistor conducts less).
// Therefore: maximum[] = black (shade 0), minimum[] = white (last shade).
//
// All shades are assumed to be equally spaced between black and
// white.  The expected normalised level for shade k is:
//   level = (N - 1 - k) / (N - 1)   → 1.0 for black, 0.0 for white
void finalizeGradientCalibration() {
  int N = calib_shade_count;

  // Assign min/max to white and black shade readings respectively
  for (int i = 0; i < NUM_SENSORS; i++) {
    // Shade 0 = black  → highest raw ADC → maximum
    line_sensors.maximum[i] = calib_readings[0][i];
    // Shade N-1 = white → lowest raw ADC  → minimum
    line_sensors.minimum[i] = calib_readings[N - 1][i];
    // scaling = range used by calcCalibratedADC()
    line_sensors.scaling[i] = line_sensors.maximum[i] - line_sensors.minimum[i];
    // Guard against zero range (identical readings)
    if (line_sensors.scaling[i] == 0) line_sensors.scaling[i] = 1;
  }

  // Print calibration summary with expected normalised shade levels
  Serial.println(F("#"));
  Serial.println(F("# === CALIBRATION RESULTS ==="));
  Serial.print(F("# Total shades recorded: "));
  Serial.println(N);
  Serial.println(F("# Expected shade spacing: equal (black=1.0, white=0.0)"));
  Serial.println(F("#"));
  Serial.println(F("# shade_index, expected_level, s0_avg, s1_avg, s2_avg, s3_avg, s4_avg"));
  for (int k = 0; k < N; k++) {
    float level = (float)(N - 1 - k) / (float)(N - 1);
    Serial.print(k);
    Serial.print(F(","));
    Serial.print(level, 4);
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(F(","));
      Serial.print(calib_readings[k][i], 1);
    }
    Serial.println();
  }

  Serial.println(F("#"));
  Serial.println(F("# Calibration values applied to line_sensors:"));
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F("#   S"));
    Serial.print(i);
    Serial.print(F(": min="));
    Serial.print(line_sensors.minimum[i], 1);
    Serial.print(F("  max="));
    Serial.print(line_sensors.maximum[i], 1);
    Serial.print(F("  range="));
    Serial.println(line_sensors.scaling[i], 1);
  }
  Serial.println(F("#"));
}

// ============================================
// SETUP
// ============================================
void setup() {
  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Buttons (active-low with internal pull-up)
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_B_PIN, INPUT_PULLUP);

  // Motors
  motors.initialise();

  // Line sensors (ADC mode with pull-ups)
  line_sensors.initialiseForADC();

  // Encoders (interrupt-driven)
  setupEncoder0();
  setupEncoder1();

  // Kinematics
  pose.initialise(0, 0, 0);

  // Serial at 115200 for high-rate data logging
  Serial.begin(115200);
  delay(1500);

  // PID controllers
  float kp = 48.0, ki = 0.5, kd = 0.0;
  left_pid.initialise(kp, ki, kd);
  right_pid.initialise(kp, ki, kd);
  pid_on = false;

  // Initialise timing
  speed_ts = millis();
  pid_ts   = millis();
  pose_ts  = millis();
  prev_e0  = count_e0;
  prev_e1  = count_e1;

  // Initialise calibration accumulator
  for (int i = 0; i < NUM_SENSORS; i++) calib_accum[i] = 0.0;

  // Welcome message — robot is already in the calibration routine
  Serial.println(F("# ========================================="));
  Serial.println(F("# LINE SENSOR SCIENTIFIC ANALYSIS"));
  Serial.println(F("# ========================================="));
  Serial.println(F("# CALIBRATION MODE — starting immediately."));
  Serial.println(F("#"));
  Serial.println(F("# Place robot on the FIRST shade (BLACK)."));
  Serial.println(F("# Press A to record. Move to next shade. Repeat."));
  Serial.println(F("# Press B when the LAST shade (WHITE) has been recorded."));
  Serial.println(F("# Any number of shades is supported."));
  Serial.println(F("#"));
  Serial.println(F("# Baud: 115200"));

  // Short beep to signal ready
  tone(BUZZER_PIN, 440, 100);
}

// ============================================
// LOOP
// ============================================
void loop() {

  // --- Continuous system updates ---
  measureSpeeds();
  updatePID();
  updatePose();

  // --- Button polling ---
  bool a_pressed = buttonA();
  bool b_pressed = buttonB();

  // --- State machine ---
  switch (test_phase) {

    // ======================================================
    // CALIBRATION — waiting for A (record shade) or B (done)
    // ======================================================
    case TEST_CALIB_WAITING:

      if (a_pressed) {
        // Guard: reached the hard cap
        if (calib_shade_count >= MAX_CALIB_SHADES) {
          Serial.println(F("# WARNING: Maximum shade count reached. Press B to finalise."));
          break;
        }

        // Start averaging samples for this shade
        for (int i = 0; i < NUM_SENSORS; i++) calib_accum[i] = 0.0;
        calib_sample_count = 0;
        test_phase = TEST_CALIB_SAMPLING;
        tone(BUZZER_PIN, 660, 50);
      }

      if (b_pressed) {
        // Require at least 2 shades: black (shade 0) and white (last shade)
        if (calib_shade_count < 2) {
          Serial.print(F("# Need at least 2 shades. Recorded so far: "));
          Serial.println(calib_shade_count);
          Serial.println(F("# Press A to record more shades, then press B."));
          tone(BUZZER_PIN, 220, 300);
          break;
        }

        // Finalise calibration and advance to Phase 2
        finalizeGradientCalibration();

        test_phase = TEST_GRADIENT_READY;
        gradient_id = 0;
        Serial.println(F("# === PHASE 2: GRADIENT TEST ==="));
        Serial.println(F("# Place robot at black end of gradient strip."));
        Serial.println(F("# Push robot slowly along gradient."));
        Serial.println(F("# Press B to start logging, B again to stop."));
        Serial.println(F("# Press A to skip to Phase 3."));
        Serial.println(F("#"));
        Serial.println(F("# HEADER: gradient_id, timestamp_ms, dist_left_mm, dist_right_mm, s0, s1, s2, s3, s4"));
        tone(BUZZER_PIN, 880, 200);
      }
      break;

    // ======================================================
    // CALIBRATION — averaging ADC samples for the current shade
    // ======================================================
    case TEST_CALIB_SAMPLING: {
      // Accumulate one reading per loop iteration
      line_sensors.readSensorsADC();
      for (int i = 0; i < NUM_SENSORS; i++) {
        calib_accum[i] += line_sensors.readings[i];
      }
      calib_sample_count++;

      // Small settling delay between ADC reads
      delay(SAMPLE_DELAY_MS);

      if (calib_sample_count >= CALIB_SAMPLES_PER_SHADE) {
        // Average the accumulated readings and store
        for (int i = 0; i < NUM_SENSORS; i++) {
          calib_readings[calib_shade_count][i] = calib_accum[i] / (float)calib_sample_count;
        }
        calib_shade_count++;

        // Report the recorded shade
        Serial.print(F("# Shade "));
        Serial.print(calib_shade_count);
        if (calib_shade_count == 1) {
          Serial.print(F(" (BLACK) recorded: "));
        } else {
          Serial.print(F(" recorded: "));
        }
        for (int i = 0; i < NUM_SENSORS; i++) {
          Serial.print(calib_readings[calib_shade_count - 1][i], 1);
          if (i < NUM_SENSORS - 1) Serial.print(F(", "));
        }
        Serial.println();
        Serial.println(F("# Move to next shade and press A.  When WHITE is done, press B."));

        test_phase = TEST_CALIB_WAITING;
        tone(BUZZER_PIN, 880, 100);
      }
      break;
    }

    // ======================================================
    // PHASE 2: GRADIENT TEST — waiting to start
    // ======================================================
    case TEST_GRADIENT_READY:
      // Button A: skip to Phase 3
      if (a_pressed) {
        Serial.println(F("# --- Phase 2 complete (skipped to Phase 3) ---"));
        test_phase = TEST_THINLINE_READY;
        tl_angle_idx = 0;
        tl_pass_id = 0;
        Serial.println(F("#"));
        Serial.println(F("# === PHASE 3: THIN LINE TEST ==="));
        Serial.println(F("# Place robot ~75mm before the thin line,"));
        Serial.println(F("# facing perpendicular to the line."));
        Serial.println(F("# Press B to start automated drive sequence."));
        Serial.println(F("# Press A to finish."));
        Serial.println(F("#"));
        Serial.println(F("# HEADER: pass_id, angle_deg, timestamp_ms, enc_left, enc_right, s0, s1, s2, s3, s4"));
        tone(BUZZER_PIN, 880, 100);
        break;
      }

      // Button B: start logging this gradient strip
      if (b_pressed) {
        gradient_id++;
        // Reset encoder counts for position reference
        noInterrupts();
        count_e0 = 0;
        count_e1 = 0;
        interrupts();
        prev_e0 = 0;
        prev_e1 = 0;
        pose.initialise(0, 0, 0);

        gradient_log_ts = millis();
        test_phase = TEST_GRADIENT_LOGGING;
        Serial.print(F("# Gradient "));
        Serial.print(gradient_id);
        Serial.println(F(" — logging started.  Push robot now.  Press B to stop."));
        tone(BUZZER_PIN, 660, 50);
      }
      break;

    // ======================================================
    // PHASE 2: GRADIENT TEST — actively logging
    // ======================================================
    case TEST_GRADIENT_LOGGING:
      // Button B: stop logging for this strip
      if (b_pressed) {
        Serial.print(F("# Gradient "));
        Serial.print(gradient_id);
        Serial.println(F(" — logging stopped.  Reposition robot, press B for next strip."));
        tone(BUZZER_PIN, 440, 100);
        test_phase = TEST_GRADIENT_READY;
        break;
      }

      // Log at fixed interval
      if (millis() - gradient_log_ts >= GRADIENT_LOG_INTERVAL_MS) {
        gradient_log_ts = millis();

        // Read sensors
        line_sensors.readSensorsADC();

        // Read encoder counts and convert to distance (mm) from strip start
        // e1 = LEFT wheel, e0 = RIGHT wheel (Encoders.h mapping)
        long e0, e1;
        noInterrupts();
        e0 = count_e0;
        e1 = count_e1;
        interrupts();
        float dist_left_mm  = countsToMM(e1);  // LEFT distance (mm)
        float dist_right_mm = countsToMM(e0);  // RIGHT distance (mm)

        // Output: gradient_id, timestamp_ms, dist_left_mm, dist_right_mm, s0-s4
        Serial.print(gradient_id);
        Serial.print(F(","));
        Serial.print(millis());
        Serial.print(F(","));
        Serial.print(dist_left_mm, 2);
        Serial.print(F(","));
        Serial.print(dist_right_mm, 2);
        Serial.print(F(","));
        printSensorCSV();
        Serial.println();
      }
      break;

    // ======================================================
    // PHASE 3: THIN LINE TEST — waiting to start
    // ======================================================
    case TEST_THINLINE_READY:
      // Button A: finish all tests
      if (a_pressed) {
        Serial.println(F("# --- Phase 3 complete ---"));
        test_phase = TEST_COMPLETE;
        tone(BUZZER_PIN, 1200, 300);
        break;
      }

      // Button B: start the crossing sequence from the current angle
      if (b_pressed) {
        // Reset encoders and pose for clean position tracking
        noInterrupts();
        count_e0 = 0;
        count_e1 = 0;
        interrupts();
        prev_e0 = 0;
        prev_e1 = 0;
        pose.initialise(0, 0, 0);

        tl_angle_idx = 0;
        tl_pass_id = 0;

        Serial.println(F("# Thin-line sequence starting."));
        Serial.print(F("# First angle: "));
        Serial.print(thinline_angles[0]);
        Serial.println(F(" deg"));

        // Enable PID for driving
        pid_on = true;
        left_pid.reset();
        right_pid.reset();

        // Record start position
        tl_start_x = pose.x;
        tl_start_y = pose.y;
        tl_log_ts = millis();
        tl_pass_id++;

        // Start driving forward
        setDemand(THINLINE_DRIVE_SPEED, THINLINE_DRIVE_SPEED);
        test_phase = TEST_THINLINE_DRIVE;
        tone(BUZZER_PIN, 660, 50);
      }
      break;

    // ======================================================
    // PHASE 3: THIN LINE — driving across the line
    // ======================================================
    case TEST_THINLINE_DRIVE: {
      // Log sensors + position at high rate
      if (millis() - tl_log_ts >= THINLINE_LOG_INTERVAL_MS) {
        tl_log_ts = millis();

        line_sensors.readSensorsADC();

        long e0, e1;
        noInterrupts();
        e0 = count_e0;
        e1 = count_e1;
        interrupts();

        // Output: pass_id, angle_deg, timestamp_ms, enc_left, enc_right, s0-s4
        Serial.print(tl_pass_id);
        Serial.print(F(","));
        Serial.print(thinline_angles[tl_angle_idx]);
        Serial.print(F(","));
        Serial.print(millis());
        Serial.print(F(","));
        Serial.print(e1);
        Serial.print(F(","));
        Serial.print(e0);
        Serial.print(F(","));
        printSensorCSV();
        Serial.println();
      }

      // Keep driving forward
      setDemand(THINLINE_DRIVE_SPEED, THINLINE_DRIVE_SPEED);

      // Check if drive distance reached
      float dist = distTo(tl_start_x, tl_start_y);
      if (dist >= THINLINE_DRIVE_DIST_MM) {
        stopMotors();
        Serial.print(F("# Pass "));
        Serial.print(tl_pass_id);
        Serial.print(F(" done ("));
        Serial.print(thinline_angles[tl_angle_idx]);
        Serial.println(F(" deg).  Reversing..."));

        // Begin reverse to starting position
        startReverse(THINLINE_DRIVE_DIST_MM);
        test_phase = TEST_THINLINE_REVERSE;
      }
      break;
    }

    // ======================================================
    // PHASE 3: THIN LINE — reversing back to start
    // ======================================================
    case TEST_THINLINE_REVERSE:
      if (updateReverse()) {
        stopMotors();
        tl_angle_idx++;

        if (tl_angle_idx >= NUM_THINLINE_ANGLES) {
          // All angles completed
          pid_on = false;
          Serial.println(F("# All thin-line angles completed."));
          Serial.println(F("# Press B to run another set, or A to finish."));
          test_phase = TEST_THINLINE_READY;
          tone(BUZZER_PIN, 880, 200);
        } else {
          // Turn to the next angle increment
          float delta_deg = (float)(thinline_angles[tl_angle_idx] - thinline_angles[tl_angle_idx - 1]);
          float delta_rad = delta_deg * PI / 180.0;

          Serial.print(F("# Turning "));
          Serial.print(delta_deg, 0);
          Serial.print(F(" deg to angle "));
          Serial.print(thinline_angles[tl_angle_idx]);
          Serial.println(F(" deg"));

          startRotation(delta_rad);
          test_phase = TEST_THINLINE_TURN;
        }
      }
      break;

    // ======================================================
    // PHASE 3: THIN LINE — turning to next angle
    // ======================================================
    case TEST_THINLINE_TURN:
      if (updateRotation()) {
        stopMotors();
        delay(200);  // Brief settle time after turn

        // Record new start position and begin next drive
        tl_start_x = pose.x;
        tl_start_y = pose.y;
        tl_pass_id++;
        tl_log_ts = millis();

        Serial.print(F("# Starting pass "));
        Serial.print(tl_pass_id);
        Serial.print(F(" at "));
        Serial.print(thinline_angles[tl_angle_idx]);
        Serial.println(F(" deg"));

        setDemand(THINLINE_DRIVE_SPEED, THINLINE_DRIVE_SPEED);
        test_phase = TEST_THINLINE_DRIVE;
      }
      break;

    // ======================================================
    // ALL TESTS COMPLETE
    // ======================================================
    case TEST_COMPLETE:
      stopMotors();
      pid_on = false;
      // Print final message once
      {
        static bool printed = false;
        if (!printed) {
          Serial.println(F("#"));
          Serial.println(F("# ========================================="));
          Serial.println(F("# ALL TESTS COMPLETE"));
          Serial.println(F("# ========================================="));
          Serial.println(F("# Disconnect serial, process data with Python."));
          printed = true;
        }
      }
      break;

  } // end switch
}
