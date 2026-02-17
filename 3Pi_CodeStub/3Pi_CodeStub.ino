/*******************************************
 *  TUNING SKETCH — Pololu 3Pi+ 32U4
 *
 *  Set TUNE_LAYER below to the layer you
 *  want to calibrate, then upload.
 *
 *  Tune in order:  1 → 3 → 0 → 4 → 5
 *******************************************/

#include "Motors.h"
#include "PID.h"
#include "Kinematics.h"
#include "Encoders.h"
#include <math.h>

// =============================================
// TUNING MODE SELECTOR
// Change this value, recompile, and upload.
//   1 = Motor dead-band measurement
//   3 = PID speed-controller gains
//   0 = Kinematics (wheel_radius, wheel_sep)
//   4 = Navigation primitives (turn, travel)
//   5 = Composite navigation (travelToXY, endurance)
// =============================================
#define TUNE_LAYER 1

// =============================================
// HARDWARE
// =============================================
#define BUZZER_PIN    6
#define BUTTON_A_PIN 14

// =============================================
// CLASS INSTANCES
// =============================================
Motors_c    motors;
Kinematics_c pose;
PID_c      left_pid;
PID_c      right_pid;

// =============================================
// SPEED MEASUREMENT
// =============================================
unsigned long speed_ts = 0;
#define SPEED_INTERVAL 5          // ms
long prev_e0 = 0, prev_e1 = 0;
float speed_e0 = 0.0;            // RIGHT wheel (counts/ms)
float speed_e1 = 0.0;            // LEFT  wheel (counts/ms)

// =============================================
// PID CONTROL
// =============================================
unsigned long pid_ts = 0;
#define PID_INTERVAL 5            // ms
float demand_left  = 0.0;
float demand_right = 0.0;
bool  use_pid = false;
float pwm_left  = 0.0;
float pwm_right = 0.0;

// =============================================
// PID GAINS — edit these during Layer 3 tuning
// =============================================
#define TUNE_KP 40.0
#define TUNE_KI  0.8
#define TUNE_KD  0.0

// =============================================
// POSE UPDATE
// =============================================
unsigned long pose_ts = 0;
#define POSE_INTERVAL 5           // ms

// =============================================
// NAVIGATION PARAMETERS — edit during Layer 4
// =============================================
#define TURN_THRESHOLD        0.01   // rad — stop turning when error < this
#define TURN_GAIN             5.0    // proportional gain for turn controller
#define MAX_TURN_SPEED        0.5    // counts/ms — cap
#define MIN_TURN_SPEED        0.15   // counts/ms — floor (dead-band)

#define TRAVEL_THRESHOLD     10.0    // mm — arrival radius
#define TRAVEL_SPEED          0.5    // counts/ms — base forward speed
#define HEADING_CORR_GAIN     0.1    // heading correction during travel
#define TRAVEL_DECEL_DIST    50.0    // mm — start decelerating
#define MIN_TRAVEL_SPEED      0.15   // counts/ms — floor

#define SETTLE_TIME_MS       50
#define SETTLE_SPEED_THRESH   0.02   // counts/ms — "stopped"

// =============================================
// NAVIGATION STATE
// =============================================
float  nav_target_angle = 0.0;
bool   nav_turning = false;

float  rot_target = 0.0;
float  rot_accumulated = 0.0;
float  rot_prev_theta = 0.0;
bool   nav_rotating = false;

float  nav_target_x = 0.0, nav_target_y = 0.0;
float  nav_init_x = 0.0,   nav_init_y = 0.0;
float  nav_init_dist = 0.0;
bool   nav_traveling = false;

enum TravelState {
  TS_IDLE, TS_SETTLING, TS_ROTATING, TS_MOVING, TS_COMPLETE
};
TravelState ts_state = TS_IDLE;
unsigned long settle_start = 0;
TravelState settle_next = TS_ROTATING;

// =============================================
// TUNING STATES — LAYER 1 (dead-band)
// =============================================
enum L1 { L1_IDLE, L1_RAMP, L1_PAUSE, L1_DONE };
L1  l1 = L1_IDLE;
int l1_phase = 0;          // 0=LF 1=LR 2=RF 3=RR
int l1_pwm   = 0;
unsigned long l1_ts = 0;
long l1_snap0 = 0, l1_snap1 = 0;

// =============================================
// TUNING STATES — LAYER 3 (PID)
// =============================================
enum L3 { L3_IDLE, L3_RUN, L3_COAST, L3_DONE };
L3  l3 = L3_IDLE;
int l3_phase = 0;           // 0=medium 1=low 2=reverse
unsigned long l3_ts = 0;
unsigned long l3_print_ts = 0;
float l3_demands[] = { 0.5, 0.15, -0.3 };
unsigned long l3_durations[] = { 5000, 5000, 3000 };

// =============================================
// TUNING STATES — LAYER 0 (kinematics)
// =============================================
enum L0 {
  L0_IDLE,
  L0_LIN_RUN, L0_LIN_DONE,
  L0_ROT_WAIT, L0_ROT_RUN, L0_ROT_DONE,
  L0_SQ_WAIT, L0_SQ_RUN, L0_SQ_DONE,
  L0_DONE
};
L0 l0 = L0_IDLE;
long l0_start_e0 = 0, l0_start_e1 = 0;
#define L0_LINEAR_TARGET_COUNTS 1000   // ~288 mm
int  l0_sq_leg = 0;
bool l0_sq_started = false;
float l0_sq_x[] = { 200, 200,   0,  0 };
float l0_sq_y[] = {   0,-200,-200,  0 };

// =============================================
// TUNING STATES — LAYER 4 (navigation)
// =============================================
enum L4 {
  L4_IDLE,
  L4_TURN_RUN, L4_TURN_DONE,
  L4_TRAV_WAIT, L4_TRAV_RUN, L4_TRAV_DONE,
  L4_SQ_WAIT, L4_SQ_RUN, L4_SQ_DONE,
  L4_DONE
};
L4   l4 = L4_IDLE;
int  l4_step = 0;
bool l4_started = false;
float l4_turns[] = { PI/2.0, PI, -PI/2.0, 0.0 };
#define L4_NUM_TURNS 4
float l4_trav_x[] = { 300,   0 };
float l4_trav_y[] = {   0,   0 };
#define L4_NUM_TRAVS 2
int  l4_sq_leg = 0;
bool l4_sq_started = false;

// =============================================
// TUNING STATES — LAYER 5 (composite)
// =============================================
enum L5 {
  L5_IDLE,
  L5_CIRCUIT, L5_CIRCUIT_DONE,
  L5_ENDURANCE_WAIT, L5_ENDURANCE, L5_ENDURANCE_DONE,
  L5_DONE
};
L5   l5 = L5_IDLE;
int  l5_wp = 0;
bool l5_started = false;
unsigned long l5_start_ms = 0;
unsigned long l5_print_ts = 0;
int  l5_lap = 0;
#define L5_NUM_WP 6
float l5_x[] = {  60.6, 260.7, 429.0, 408.8, 354.3, 156.7 };
float l5_y[] = { 262.9, 211.4, 267.7, 113.9, -10.9,  83.9 };
#define L5_ENDURANCE_MS 240000UL   // 4 minutes

// =============================================
// BUTTON HELPER (blocking debounce — only
// call when robot is stationary)
// =============================================
bool waitButton() {
  if (digitalRead(BUTTON_A_PIN) == LOW) {
    delay(50);
    while (digitalRead(BUTTON_A_PIN) == LOW) {}
    delay(50);
    return true;
  }
  return false;
}

// =================================================================
//  SETUP
// =================================================================
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  pose.initialise(0, 0, 0);

  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("========================================");
  Serial.print  ("  TUNING MODE — LAYER ");
  Serial.println(TUNE_LAYER);
  Serial.println("========================================");
  Serial.println("Press BUTTON A to begin.");

  // Wait for initial button press
  while (digitalRead(BUTTON_A_PIN) != LOW) {}
  delay(50);
  while (digitalRead(BUTTON_A_PIN) == LOW) {}
  delay(50);
  tone(BUZZER_PIN, 880, 100);

  // Initialise PID (needed for layers 3+)
  left_pid.initialise(TUNE_KP, TUNE_KI, TUNE_KD);
  right_pid.initialise(TUNE_KP, TUNE_KI, TUNE_KD);

  // Capture timestamps
  prev_e0 = count_e0;
  prev_e1 = count_e1;
  speed_ts  = millis();
  pid_ts    = millis();
  pose_ts   = millis();

  Serial.println("*** STARTED ***");
  Serial.println();
}

// =================================================================
//  CORE SYSTEM FUNCTIONS
// =================================================================

void measureSpeeds() {
  unsigned long now = millis();
  unsigned long dt  = now - speed_ts;
  if (dt >= SPEED_INTERVAL) {
    long le0, le1;
    noInterrupts();
    le0 = count_e0;
    le1 = count_e1;
    interrupts();

    speed_e0 = (float)(le0 - prev_e0) / (float)dt;
    speed_e1 = (float)(le1 - prev_e1) / (float)dt;
    prev_e0 = le0;
    prev_e1 = le1;
    speed_ts = now;
  }
}

void updatePID() {
  unsigned long now = millis();
  if (now - pid_ts < PID_INTERVAL) return;

  if (use_pid) {
    if (demand_left == 0.0) {
      pwm_left = 0.0;
      left_pid.reset();
    } else {
      pwm_left = left_pid.update(demand_left, speed_e1);
    }
    if (demand_right == 0.0) {
      pwm_right = 0.0;
      right_pid.reset();
    } else {
      pwm_right = right_pid.update(demand_right, speed_e0);
    }
    motors.setPWM(pwm_left, pwm_right);
  } else {
    left_pid.reset();
    right_pid.reset();
  }
  pid_ts = now;
}

void updatePose() {
  unsigned long now = millis();
  if (now - pose_ts >= POSE_INTERVAL) {
    pose.update();
    pose_ts = now;
  }
}

// ---- helpers ----

void setDemands(float l, float r) { demand_left = l; demand_right = r; }

void stopPID() {
  setDemands(0, 0);
  motors.setPWM(0, 0);
  left_pid.reset();
  right_pid.reset();
}

void resetPose() {
  pose.initialise(0, 0, 0);
}

void beep(int freq, int ms) { tone(BUZZER_PIN, freq, ms); }

// Print pose in a compact, readable format
void printPose(const char* label) {
  Serial.print(label);
  Serial.print("  x="); Serial.print(pose.x, 1);
  Serial.print("  y="); Serial.print(pose.y, 1);
  Serial.print("  th="); Serial.print(pose.theta * 180.0 / PI, 1);
  Serial.println("°");
}

// =================================================================
//  NAVIGATION FUNCTIONS (identical logic to main codebase)
// =================================================================

float angleDiff(float target, float current) {
  return atan2(sin(target - current), cos(target - current));
}

float angleToPoint(float tx, float ty) {
  return atan2(ty - pose.y, tx - pose.x);
}

float distToPoint(float tx, float ty) {
  float dx = tx - pose.x, dy = ty - pose.y;
  return sqrt(dx*dx + dy*dy);
}

// ---- TURN (absolute heading) ----

void setTurn(float angle) {
  nav_target_angle = angle;
  nav_turning = true;
}

bool checkTurn() {
  if (!nav_turning) return true;
  float err = angleDiff(nav_target_angle, pose.theta);
  if (fabs(err) < TURN_THRESHOLD) {
    stopPID(); nav_turning = false; return true;
  }
  float spd = err * TURN_GAIN;
  if (spd >  MAX_TURN_SPEED) spd =  MAX_TURN_SPEED;
  if (spd < -MAX_TURN_SPEED) spd = -MAX_TURN_SPEED;
  if (fabs(spd) < MIN_TURN_SPEED)
    spd = (err > 0) ? MIN_TURN_SPEED : -MIN_TURN_SPEED;
  setDemands(-spd, spd);
  return false;
}

// ---- ROTATION (relative, multi-rev) ----

void setRotation(float radians) {
  rot_target = radians;
  rot_accumulated = 0.0;
  rot_prev_theta = pose.theta;
  nav_rotating = true;
}

bool checkRotation() {
  if (!nav_rotating) return true;
  float delta = angleDiff(pose.theta, rot_prev_theta);
  rot_accumulated += delta;
  rot_prev_theta = pose.theta;
  float remaining = rot_target - rot_accumulated;
  if (fabs(remaining) < TURN_THRESHOLD) {
    stopPID(); nav_rotating = false; return true;
  }
  float spd = remaining * TURN_GAIN;
  if (spd >  MAX_TURN_SPEED) spd =  MAX_TURN_SPEED;
  if (spd < -MAX_TURN_SPEED) spd = -MAX_TURN_SPEED;
  if (fabs(spd) < MIN_TURN_SPEED)
    spd = (remaining > 0) ? MIN_TURN_SPEED : -MIN_TURN_SPEED;
  setDemands(-spd, spd);
  return false;
}

// ---- TRAVEL (straight to point) ----

void setTravel(float x, float y) {
  nav_target_x = x;  nav_target_y = y;
  nav_init_x = pose.x; nav_init_y = pose.y;
  nav_init_dist = distToPoint(x, y);
  nav_traveling = true;
}

bool checkTravel() {
  if (!nav_traveling) return true;
  float dist = distToPoint(nav_target_x, nav_target_y);
  float traveled = distToPoint(nav_init_x, nav_init_y);
  if (dist < TRAVEL_THRESHOLD || traveled >= nav_init_dist) {
    stopPID(); nav_traveling = false; return true;
  }
  float heading_err = angleDiff(
    angleToPoint(nav_target_x, nav_target_y), pose.theta);
  float fwd = TRAVEL_SPEED;
  if (dist < TRAVEL_DECEL_DIST) {
    fwd = TRAVEL_SPEED * (dist / TRAVEL_DECEL_DIST);
    if (fwd < MIN_TRAVEL_SPEED) fwd = MIN_TRAVEL_SPEED;
  }
  float corr = heading_err * HEADING_CORR_GAIN;
  setDemands(fwd + corr, fwd - corr);
  return false;
}

// ---- TRAVEL-TO-XY (turn then travel, with settling) ----

void travelToXY(float x, float y) {
  nav_target_x = x; nav_target_y = y;
  stopPID();
  settle_start = millis();
  settle_next = TS_ROTATING;
  ts_state = TS_SETTLING;
}

bool updateTravelToXY() {
  switch (ts_state) {
    case TS_IDLE: return true;

    case TS_SETTLING: {
      bool t_ok = (millis() - settle_start >= SETTLE_TIME_MS);
      bool s_ok = (fabs(speed_e0) < SETTLE_SPEED_THRESH &&
                   fabs(speed_e1) < SETTLE_SPEED_THRESH);
      if (t_ok && s_ok) {
        left_pid.reset(); right_pid.reset();
        ts_state = settle_next;
      }
      return false;
    }

    case TS_ROTATING:
      if (!nav_turning) setTurn(angleToPoint(nav_target_x, nav_target_y));
      if (checkTurn()) {
        stopPID();
        settle_start = millis();
        settle_next = TS_MOVING;
        ts_state = TS_SETTLING;
      }
      return false;

    case TS_MOVING:
      if (!nav_traveling) setTravel(nav_target_x, nav_target_y);
      if (checkTravel()) { ts_state = TS_COMPLETE; }
      return false;

    case TS_COMPLETE:
      ts_state = TS_IDLE;
      return true;
  }
  return true;
}

// =================================================================
//  LAYER 1 — MOTOR DEAD-BAND MEASUREMENT
// =================================================================
void tuneLayer1() {
  switch (l1) {

    case L1_IDLE: {
      const char* names[] = {"Left FWD","Left REV","Right FWD","Right REV"};
      Serial.println("=== LAYER 1: DEAD-BAND MEASUREMENT ===");
      Serial.println("Ramps PWM 0→25 for each motor/direction.");
      Serial.println("Note the PWM where DELTA first becomes non-zero.");
      Serial.println();
      Serial.print("Starting: "); Serial.println(names[l1_phase]);
      l1_pwm = 0;
      l1_snap0 = count_e0; l1_snap1 = count_e1;
      l1_ts = millis();
      l1 = L1_RAMP;
      break;
    }

    case L1_RAMP:
      if (millis() - l1_ts >= 500) {
        // Read encoder delta since last step
        long le0, le1;
        noInterrupts(); le0 = count_e0; le1 = count_e1; interrupts();

        long delta;
        if (l1_phase < 2) delta = abs(le1 - l1_snap1);  // left motor → e1
        else              delta = abs(le0 - l1_snap0);  // right motor → e0

        // Print result for this PWM step
        const char* names[] = {"L_FWD","L_REV","R_FWD","R_REV"};
        Serial.print(names[l1_phase]);
        Serial.print(" | PWM:");
        if (l1_pwm < 10) Serial.print(' ');
        Serial.print(l1_pwm);
        Serial.print(" | delta:");
        Serial.print(delta);
        if (delta > 0) Serial.print("  <<< MOVING");
        Serial.println();

        l1_snap0 = le0; l1_snap1 = le1;
        l1_pwm++;

        if (l1_pwm > 25) {
          // Done with this phase
          analogWrite(L_PWM, 0); analogWrite(R_PWM, 0);
          l1_phase++;
          if (l1_phase > 3) { l1 = L1_DONE; return; }
          l1 = L1_PAUSE;
          l1_ts = millis();
          return;
        }

        // Apply raw PWM (bypassing dead-band compensation)
        switch (l1_phase) {
          case 0: digitalWrite(L_DIR, FWD); analogWrite(L_PWM, l1_pwm); break;
          case 1: digitalWrite(L_DIR, REV); analogWrite(L_PWM, l1_pwm); break;
          case 2: digitalWrite(R_DIR, FWD); analogWrite(R_PWM, l1_pwm); break;
          case 3: digitalWrite(R_DIR, REV); analogWrite(R_PWM, l1_pwm); break;
        }
        l1_ts = millis();
      }
      break;

    case L1_PAUSE:
      if (millis() - l1_ts >= 1500) {
        const char* names[] = {"Left FWD","Left REV","Right FWD","Right REV"};
        Serial.println();
        Serial.print("Starting: "); Serial.println(names[l1_phase]);
        l1_pwm = 0;
        l1_snap0 = count_e0; l1_snap1 = count_e1;
        l1_ts = millis();
        l1 = L1_RAMP;
      }
      break;

    case L1_DONE:
      Serial.println();
      Serial.println("=== LAYER 1 COMPLETE ===");
      Serial.println("Update dead-band defines in Motors.h:");
      Serial.println("  L_DEADBAND_FWD, L_DEADBAND_REV");
      Serial.println("  R_DEADBAND_FWD, R_DEADBAND_REV");
      Serial.println("Then set TUNE_LAYER to 3 and re-upload.");
      l1 = (L1)99; // park
      break;
  }
}

// =================================================================
//  LAYER 3 — PID GAIN TUNING
// =================================================================
void tuneLayer3() {
  switch (l3) {

    case L3_IDLE:
      Serial.println("=== LAYER 3: PID STEP-RESPONSE TEST ===");
      Serial.println("Open Serial Plotter (115200 baud) to see live traces.");
      Serial.println("Phase 0: demand=0.50 c/ms (5 s)");
      Serial.println("Phase 1: demand=0.15 c/ms (5 s)");
      Serial.println("Phase 2: demand=-0.30 c/ms (3 s, reverse)");
      Serial.println();
      Serial.println("Adjust TUNE_KP / TUNE_KI / TUNE_KD at top of file,");
      Serial.println("recompile and repeat until satisfied.");
      Serial.println();
      Serial.println("Press BUTTON A to run all 3 phases.");
      l3 = (L3)10; // wait-for-button sub-state
      break;

    case (L3)10: // wait for button
      if (waitButton()) {
        use_pid = true;
        left_pid.reset(); right_pid.reset();
        l3_phase = 0;
        l3 = L3_RUN;
        l3_ts = millis();
        l3_print_ts = millis();
        Serial.println("demand,left,right");
        setDemands(l3_demands[0], l3_demands[0]);
      }
      break;

    case L3_RUN: {
      float d = l3_demands[l3_phase];
      // Print for Serial Plotter every 20 ms
      if (millis() - l3_print_ts >= 20) {
        Serial.print("demand:"); Serial.print(d, 3);
        Serial.print(",left:");  Serial.print(speed_e1, 3);
        Serial.print(",right:"); Serial.println(speed_e0, 3);
        l3_print_ts = millis();
      }
      if (millis() - l3_ts >= l3_durations[l3_phase]) {
        stopPID();
        l3 = L3_COAST;
        l3_ts = millis();
      }
      break;
    }

    case L3_COAST:
      if (millis() - l3_ts >= 2000) {
        l3_phase++;
        if (l3_phase >= 3) {
          l3 = L3_DONE;
        } else {
          left_pid.reset(); right_pid.reset();
          setDemands(l3_demands[l3_phase], l3_demands[l3_phase]);
          l3_ts = millis();
          l3_print_ts = millis();
          l3 = L3_RUN;
        }
      }
      break;

    case L3_DONE:
      Serial.println();
      Serial.println("=== LAYER 3 COMPLETE ===");
      Serial.println("Look for:");
      Serial.println("  - Fast rise to demand without large overshoot (Kp)");
      Serial.println("  - No steady-state offset from demand line (Ki)");
      Serial.println("  - No sustained oscillation (reduce Kp or add Kd)");
      Serial.println();
      Serial.println("Satisfied? Set TUNE_LAYER to 0 and re-upload.");
      Serial.println("Press BUTTON A to repeat test.");
      l3 = (L3)10; // back to wait-for-button
      break;
  }
}

// =================================================================
//  LAYER 0 — KINEMATICS CALIBRATION
// =================================================================
void tuneLayer0() {
  switch (l0) {

    // ---------- instructions ----------
    case L0_IDLE:
      Serial.println("=== LAYER 0: KINEMATICS CALIBRATION ===");
      Serial.println();
      Serial.println("--- SUB-TEST A: LINEAR (wheel_radius) ---");
      Serial.println("Robot drives straight for 1000 encoder counts (~288 mm).");
      Serial.println("Measure actual distance with a ruler.");
      Serial.println();
      Serial.println("  Formula:  new_radius = old_radius x (measured / pose.x)");
      Serial.println();
      Serial.println("Place robot at a ruler's 0 mark, facing along it.");
      Serial.println("Press BUTTON A to drive.");
      l0 = (L0)10;
      break;

    case (L0)10: // wait button → linear
      if (waitButton()) {
        resetPose();
        use_pid = true;
        left_pid.reset(); right_pid.reset();
        noInterrupts();
        l0_start_e0 = count_e0;
        l0_start_e1 = count_e1;
        interrupts();
        setDemands(0.3, 0.3);
        beep(660, 100);
        l0 = L0_LIN_RUN;
      }
      break;

    // ---------- linear drive ----------
    case L0_LIN_RUN: {
      long le0, le1;
      noInterrupts(); le0 = count_e0; le1 = count_e1; interrupts();
      long avg = ((le0 - l0_start_e0) + (le1 - l0_start_e1)) / 2;
      if (avg >= L0_LINEAR_TARGET_COUNTS) {
        stopPID();
        beep(880, 200);
        l0 = L0_LIN_DONE;
      }
      break;
    }

    case L0_LIN_DONE:
      Serial.println();
      Serial.println("--- LINEAR RESULT ---");
      printPose("Reported");
      Serial.print("Current wheel_radius = ");
      Serial.print(wheel_radius, 3); Serial.println(" mm");
      Serial.println();
      Serial.println("Measure actual distance with ruler, then:");
      Serial.println("  new_radius = "); Serial.print(wheel_radius, 3);
      Serial.println(" x (actual_mm / "); Serial.print(pose.x, 1);
      Serial.println(")");
      Serial.println();
      Serial.println("Update wheel_radius in Kinematics.h, recompile,");
      Serial.println("and repeat until pose.x matches the ruler.");
      Serial.println();
      Serial.println("Press BUTTON A for rotation test.");
      l0 = L0_ROT_WAIT;
      break;

    // ---------- rotation ----------
    case L0_ROT_WAIT:
      if (waitButton()) {
        Serial.println();
        Serial.println("--- SUB-TEST B: ROTATION (wheel_sep) ---");
        Serial.println("Robot will rotate 4 full turns (CCW).");
        Serial.println();
        Serial.println("  1. Place tape on floor aligned with robot front");
        Serial.println("  2. After 4 turns, check if robot faces the tape");
        Serial.println();
        Serial.println("  Under-rotated → wheel_sep too SMALL → increase");
        Serial.println("  Over-rotated  → wheel_sep too LARGE → decrease");
        Serial.println();
        Serial.println("  Formula: new_sep = old_sep x (4.0 / actual_turns)");
        Serial.println("  where actual_turns = 4 + (overshoot_deg / 360)");
        Serial.println();
        Serial.println("Press BUTTON A to start rotation.");
        l0 = (L0)11;
      }
      break;

    case (L0)11: // wait button → rotate
      if (waitButton()) {
        resetPose();
        use_pid = true;
        left_pid.reset(); right_pid.reset();
        setRotation(4.0 * 2.0 * PI);
        beep(660, 100);
        l0 = L0_ROT_RUN;
      }
      break;

    case L0_ROT_RUN:
      if (checkRotation()) {
        stopPID();
        beep(880, 200);
        l0 = L0_ROT_DONE;
      }
      break;

    case L0_ROT_DONE:
      Serial.println();
      Serial.println("--- ROTATION RESULT ---");
      printPose("Reported");
      Serial.print("Accumulated rotation = ");
      Serial.print(rot_accumulated * 180.0 / PI, 1);
      Serial.println("°");
      Serial.print("Current wheel_sep = ");
      Serial.print(wheel_sep, 3); Serial.println(" mm");
      Serial.println();
      Serial.println("Check: does robot face the tape mark?");
      Serial.println("Update wheel_sep in Kinematics.h if needed.");
      Serial.println();
      Serial.println("Press BUTTON A for square test.");
      l0 = L0_SQ_WAIT;
      break;

    // ---------- square test ----------
    case L0_SQ_WAIT:
      if (waitButton()) {
        Serial.println();
        Serial.println("--- SUB-TEST C: SQUARE (combined validation) ---");
        Serial.println("Robot drives a 200 mm square and returns to start.");
        Serial.println("Final pose should be near (0, 0, 0).");
        Serial.println();
        resetPose();
        use_pid = true;
        left_pid.reset(); right_pid.reset();
        l0_sq_leg = 0;
        l0_sq_started = false;
        beep(660, 100);
        l0 = L0_SQ_RUN;
      }
      break;

    case L0_SQ_RUN:
      if (!l0_sq_started) {
        travelToXY(l0_sq_x[l0_sq_leg], l0_sq_y[l0_sq_leg]);
        l0_sq_started = true;
      }
      if (updateTravelToXY()) {
        Serial.print("  Corner "); Serial.print(l0_sq_leg + 1);
        printPose(": ");
        l0_sq_leg++;
        l0_sq_started = false;
        if (l0_sq_leg >= 4) {
          stopPID();
          beep(880, 200);
          l0 = L0_SQ_DONE;
        }
      }
      break;

    case L0_SQ_DONE: {
      Serial.println();
      Serial.println("--- SQUARE RESULT ---");
      printPose("Final");
      float err = sqrt(pose.x*pose.x + pose.y*pose.y);
      Serial.print("Position error = "); Serial.print(err, 1); Serial.println(" mm");
      Serial.print("Heading error  = "); Serial.print(pose.theta * 180.0/PI, 1);
      Serial.println("°");
      Serial.println();
      Serial.println("Target: position error < 20 mm, heading error < 5°.");
      Serial.println("If poor, re-run sub-tests A and B.");
      Serial.println();
      Serial.println("Press BUTTON A to restart Layer 0 tests.");
      l0 = L0_DONE;
      break;
    }

    case L0_DONE:
      if (waitButton()) {
        l0 = L0_IDLE;
      }
      break;
  }
}

// =================================================================
//  LAYER 4 — NAVIGATION PRIMITIVES
// =================================================================
void tuneLayer4() {
  switch (l4) {

    case L4_IDLE:
      Serial.println("=== LAYER 4: NAVIGATION PRIMITIVES ===");
      Serial.println();
      Serial.println("Sub-test A: 4 absolute turns (90, 180, -90, 0)");
      Serial.println("Sub-test B: 2 travel legs ((300,0) then (0,0))");
      Serial.println("Sub-test C: 200 mm square validation");
      Serial.println();
      Serial.println("Press BUTTON A to start turn test.");
      l4 = (L4)10;
      break;

    case (L4)10:
      if (waitButton()) {
        resetPose();
        use_pid = true;
        left_pid.reset(); right_pid.reset();
        l4_step = 0;
        l4_started = false;
        beep(660, 100);
        l4 = L4_TURN_RUN;
      }
      break;

    // ---------- turn test ----------
    case L4_TURN_RUN:
      if (!l4_started) {
        setTurn(l4_turns[l4_step]);
        l4_started = true;
        Serial.print("Turning to ");
        Serial.print(l4_turns[l4_step] * 180.0/PI, 1);
        Serial.println("°...");
      }
      if (checkTurn()) {
        float err = angleDiff(l4_turns[l4_step], pose.theta);
        Serial.print("  Turn "); Serial.print(l4_step + 1);
        Serial.print(": target="); Serial.print(l4_turns[l4_step]*180.0/PI, 1);
        Serial.print("°  actual="); Serial.print(pose.theta*180.0/PI, 1);
        Serial.print("°  error="); Serial.print(err*180.0/PI, 2);
        Serial.println("°");
        l4_step++;
        l4_started = false;
        if (l4_step >= L4_NUM_TURNS) {
          stopPID(); beep(880, 200);
          l4 = L4_TURN_DONE;
        }
      }
      break;

    case L4_TURN_DONE:
      Serial.println();
      Serial.println("Turn test complete.");
      Serial.println("Target: each turn error < 2° (0.035 rad).");
      Serial.println("If poor: adjust TURN_GAIN, MIN_TURN_SPEED, TURN_THRESHOLD.");
      Serial.println();
      Serial.println("Press BUTTON A for travel test.");
      l4 = L4_TRAV_WAIT;
      break;

    // ---------- travel test ----------
    case L4_TRAV_WAIT:
      if (waitButton()) {
        resetPose();
        left_pid.reset(); right_pid.reset();
        l4_step = 0;
        l4_started = false;
        beep(660, 100);
        l4 = L4_TRAV_RUN;
      }
      break;

    case L4_TRAV_RUN:
      if (!l4_started) {
        travelToXY(l4_trav_x[l4_step], l4_trav_y[l4_step]);
        l4_started = true;
        Serial.print("Traveling to (");
        Serial.print(l4_trav_x[l4_step], 0);
        Serial.print(", ");
        Serial.print(l4_trav_y[l4_step], 0);
        Serial.println(")...");
      }
      if (updateTravelToXY()) {
        float ex = pose.x - l4_trav_x[l4_step];
        float ey = pose.y - l4_trav_y[l4_step];
        float err = sqrt(ex*ex + ey*ey);
        Serial.print("  Leg "); Serial.print(l4_step + 1);
        printPose(": ");
        Serial.print("  Position error = "); Serial.print(err, 1);
        Serial.println(" mm");
        l4_step++;
        l4_started = false;
        if (l4_step >= L4_NUM_TRAVS) {
          stopPID(); beep(880, 200);
          l4 = L4_TRAV_DONE;
        }
      }
      break;

    case L4_TRAV_DONE:
      Serial.println();
      Serial.println("Travel test complete.");
      Serial.println("Target: arrival error < 15 mm.");
      Serial.println("If poor: adjust TRAVEL_SPEED, HEADING_CORR_GAIN,");
      Serial.println("         TRAVEL_DECEL_DIST, TRAVEL_THRESHOLD.");
      Serial.println();
      Serial.println("Press BUTTON A for square test.");
      l4 = L4_SQ_WAIT;
      break;

    // ---------- square test ----------
    case L4_SQ_WAIT:
      if (waitButton()) {
        resetPose();
        left_pid.reset(); right_pid.reset();
        l4_sq_leg = 0;
        l4_sq_started = false;
        beep(660, 100);
        l4 = L4_SQ_RUN;
      }
      break;

    case L4_SQ_RUN:
      if (!l4_sq_started) {
        travelToXY(l0_sq_x[l4_sq_leg], l0_sq_y[l4_sq_leg]);
        l4_sq_started = true;
      }
      if (updateTravelToXY()) {
        Serial.print("  Corner "); Serial.print(l4_sq_leg + 1);
        printPose(": ");
        l4_sq_leg++;
        l4_sq_started = false;
        if (l4_sq_leg >= 4) {
          stopPID(); beep(880, 200);
          l4 = L4_SQ_DONE;
        }
      }
      break;

    case L4_SQ_DONE: {
      Serial.println();
      Serial.println("--- SQUARE RESULT ---");
      printPose("Final");
      float err = sqrt(pose.x*pose.x + pose.y*pose.y);
      Serial.print("Return error = "); Serial.print(err, 1); Serial.println(" mm");
      Serial.println("Target: < 20 mm and < 5° heading error.");
      Serial.println();
      Serial.println("Press BUTTON A to restart Layer 4.");
      l4 = L4_DONE;
      break;
    }

    case L4_DONE:
      if (waitButton()) l4 = L4_IDLE;
      break;
  }
}

// =================================================================
//  LAYER 5 — COMPOSITE NAVIGATION & ENDURANCE
// =================================================================
void tuneLayer5() {
  switch (l5) {

    case L5_IDLE:
      Serial.println("=== LAYER 5: COMPOSITE NAVIGATION ===");
      Serial.println();
      Serial.println("Sub-test A: Navigate all 6 waypoints + return home.");
      Serial.println("Sub-test B: 4-minute endurance (repeated laps).");
      Serial.println();
      Serial.println("Press BUTTON A to start waypoint circuit.");
      l5 = (L5)10;
      break;

    case (L5)10:
      if (waitButton()) {
        resetPose();
        use_pid = true;
        left_pid.reset(); right_pid.reset();
        l5_wp = 0;
        l5_started = false;
        l5_start_ms = millis();
        beep(660, 100);
        Serial.println("--- WAYPOINT CIRCUIT ---");
        l5 = L5_CIRCUIT;
      }
      break;

    // ---------- waypoint circuit ----------
    case L5_CIRCUIT:
      if (!l5_started) {
        if (l5_wp < L5_NUM_WP) {
          travelToXY(l5_x[l5_wp], l5_y[l5_wp]);
          Serial.print("  → WP"); Serial.print(l5_wp + 1);
          Serial.print(" ("); Serial.print(l5_x[l5_wp], 0);
          Serial.print(", "); Serial.print(l5_y[l5_wp], 0);
          Serial.println(")");
        } else {
          // Return home
          travelToXY(0, 0);
          Serial.println("  → HOME (0, 0)");
        }
        l5_started = true;
      }
      if (updateTravelToXY()) {
        if (l5_wp < L5_NUM_WP) {
          float ex = pose.x - l5_x[l5_wp];
          float ey = pose.y - l5_y[l5_wp];
          float err = sqrt(ex*ex + ey*ey);
          Serial.print("    Arrived. err="); Serial.print(err, 1);
          Serial.print("mm ");
          printPose("");
        }
        l5_wp++;
        l5_started = false;
        if (l5_wp > L5_NUM_WP) { // all WPs + home done
          stopPID(); beep(880, 200);
          l5 = L5_CIRCUIT_DONE;
        }
      }
      break;

    case L5_CIRCUIT_DONE: {
      unsigned long elapsed = millis() - l5_start_ms;
      Serial.println();
      Serial.println("--- CIRCUIT RESULT ---");
      printPose("Final");
      float err = sqrt(pose.x*pose.x + pose.y*pose.y);
      Serial.print("Home error = "); Serial.print(err, 1); Serial.println(" mm");
      Serial.print("Circuit time = "); Serial.print(elapsed / 1000);
      Serial.println(" s");
      Serial.println();
      Serial.println("Target: home error < 30 mm, time < 60 s.");
      Serial.println();
      Serial.println("Press BUTTON A for 4-minute endurance test.");
      l5 = L5_ENDURANCE_WAIT;
      break;
    }

    // ---------- endurance ----------
    case L5_ENDURANCE_WAIT:
      if (waitButton()) {
        resetPose();
        left_pid.reset(); right_pid.reset();
        l5_wp = 0;
        l5_started = false;
        l5_start_ms = millis();
        l5_print_ts = millis();
        l5_lap = 0;
        beep(660, 100);
        Serial.println("--- 4-MINUTE ENDURANCE TEST ---");
        Serial.println("Printing pose every 30 seconds.");
        Serial.println();
        l5 = L5_ENDURANCE;
      }
      break;

    case L5_ENDURANCE: {
      unsigned long elapsed = millis() - l5_start_ms;

      // Periodic pose print
      if (millis() - l5_print_ts >= 30000) {
        Serial.print("["); Serial.print(elapsed / 1000); Serial.print("s] ");
        Serial.print("Lap "); Serial.print(l5_lap);
        printPose("  ");
        l5_print_ts = millis();
      }

      // Timer expired → go home
      if (elapsed >= L5_ENDURANCE_MS) {
        Serial.println("*** 4 MINUTES — returning home ***");
        travelToXY(0, 0);
        l5_started = true;
        l5 = L5_ENDURANCE_DONE;
        break;
      }

      // Navigate waypoints in a loop
      if (!l5_started) {
        travelToXY(l5_x[l5_wp], l5_y[l5_wp]);
        l5_started = true;
      }
      if (updateTravelToXY()) {
        l5_wp++;
        l5_started = false;
        if (l5_wp >= L5_NUM_WP) {
          l5_wp = 0;
          l5_lap++;
          // Return home between laps
          travelToXY(0, 0);
          l5_started = true;
        }
      }
      break;
    }

    case L5_ENDURANCE_DONE:
      if (updateTravelToXY()) {
        stopPID(); beep(880, 400);
        Serial.println();
        Serial.println("--- ENDURANCE RESULT ---");
        printPose("Final");
        float err = sqrt(pose.x*pose.x + pose.y*pose.y);
        Serial.print("Home error  = "); Serial.print(err, 1); Serial.println(" mm");
        Serial.print("Laps done   = "); Serial.println(l5_lap);
        unsigned long total = millis() - l5_start_ms;
        Serial.print("Total time  = "); Serial.print(total / 1000); Serial.println(" s");
        Serial.println();
        Serial.println("Target: home error < 30 mm after full 4 minutes.");
        Serial.println("If drift is large, re-calibrate Layer 0 (kinematics).");
        Serial.println();
        Serial.println("Press BUTTON A to restart Layer 5.");
        l5 = L5_DONE;
      }
      break;

    case L5_DONE:
      if (waitButton()) l5 = L5_IDLE;
      break;
  }
}

// =================================================================
//  LOOP
// =================================================================
void loop() {
  // Always measure speeds and update pose
  measureSpeeds();
  updatePose();

  // Only run PID for layers that need it
  if (TUNE_LAYER != 1) updatePID();

  switch (TUNE_LAYER) {
    case 1: tuneLayer1(); break;
    case 3: tuneLayer3(); break;
    case 0: tuneLayer0(); break;
    case 4: tuneLayer4(); break;
    case 5: tuneLayer5(); break;
  }
}
