/******************************************
  12345
	______      _           _   _              
	| ___ \    | |         | | (_)             
	| |_/ /___ | |__   ___ | |_ _  ___ ___     
	|    // _ \| '_ \ / _ \| __| |/ __/ __|    
	| |\ \ (_) | |_) | (_) | |_| | (__\__ \    
	\_| \_\___/|_.__/ \___/ \__|_|\___|___/    
											   
											   
	 _____      _                              
	/  ___|    (_)                       ___   
	\ `--.  ___ _  ___ _ __   ___ ___   ( _ )  
	 `--. \/ __| |/ _ \ '_ \ / __/ _ \  / _ \/\
	/\__/ / (__| |  __/ | | | (_|  __/ | (_>  <
	\____/ \___|_|\___|_| |_|\___\___|  \___/\/
											   
											   
	 _____           _                         
	/  ___|         | |                        
	\ `--. _   _ ___| |_ ___ _ __ ___  ___     
	 `--. \ | | / __| __/ _ \ '_ ` _ \/ __|    
	/\__/ / |_| \__ \ ||  __/ | | | | \__ \    
	\____/ \__, |___/\__\___|_| |_| |_|___/    
			__/ |                              
		   |___/                               


 SEMTM0042/43: University of Bristol.
 https://github.com/paulodowd/SEMTM0042_43
*******************************************/


// These #include commands essentially "copy and paste" 
// the above .h files (tabs above) into your code here.

// Labsheet 1: Build a class to operate your motors.
#include "Motors.h"         

// Labsheet 1: Tune PID to control wheel speed.
#include "PID.h"            

// Labsheet 2: Build a class to read the line sensors.
#include "LineSensors.h"    

// Labsheet 2: Operate the magnetometer to detect the 
//             puck.  You will need to follow the steps
//             to install the software library.
#include "Magnetometer.h" // Labsheet 2

// Labsheet 3: Calibrate and use kinematics to allow 
//             your robot to turn and travel between
//             locations.
#include "Kinematics.h"  

// Encoders.h does not need modifying.
#include "Encoders.h"     // For encoder counts


// If you want to use one of the Display modules for
// your robot, please read Supplementary Labsheet 4.
// You can only use one of the following.  Uncomment
// the correct one for your display module.  If you
// have not followed Supplementary Labsheet 4, these 
// will cause a compile error in your code.  

// Uncomment the next two lines if you will use
// the GREEN display with 2 rows of pins.
//#include "lcd.h"
//LCD_c display(0, 1, 14, 17, 13, 30);

// Uncomment the next two lines if you will use
// the BLUE display with 1 row of pins.
#include "oled.h"
OLED_c display(1, 30, 0, 17, 13);


// Used for Labsheet 0. Defines which pin the buzzer
// is attached to. #define works like a
// find-and-replace through your code.
// See Labsheet 0.
#define BUZZER_PIN 6

// Button A pin on Pololu 3Pi+ 32U4
// Button is active-low (pressed = LOW)
#define BUTTON_A_PIN 14

// Instance of a class to operate motors.
// A class is like a template, and we name
// an "instance" to use.
// You can recognise a class type in the
// code by the convention of "_c"
// You will need to complete the class.
// See Labsheet 1.
Motors_c motors;    

// Instance of a class to operate the line
// sensors to measure the surface reflectance.
// You will need to complete the class.
// See Labsheet 2.
LineSensors_c line_sensors; 

// Instance of a class to operate the magnetometer.
// Completing the class is a later exercise in 
// Labsheet 2, so you can leave this commented
// out.
// See Labsheet 2.
Magnetometer_c magnetometer;

// Instance of a class to estimate the pose
// of the robot.  You will need to calibrate
// this, and potentially improve it.
// See Labsheet 3.
Kinematics_c pose;

// ============================================
// STAGE 4: PID Speed Controllers
// ============================================
// Two PID controllers - one for each motor
// These will maintain constant wheel speeds by
// automatically adjusting PWM output
PID_c left_pid;   // PID controller for left motor
PID_c right_pid;  // PID controller for right motor

// ============================================
// STAGE 3: ENCODER READING & SPEED MEASUREMENT VARIABLES
// ============================================

// Speed measurement timing
unsigned long speed_measurement_ts = 0;
#define SPEED_MEASUREMENT_INTERVAL 20  // Measure speed every 5ms

// Previous encoder counts (for calculating change over time)
long prev_count_e0 = 0;
long prev_count_e1 = 0;

// Calculated wheel speeds (in encoder counts per millisecond)
// NOTE: From Encoders.h, e0 = RIGHT wheel, e1 = LEFT wheel
float speed_e0 = 0.0;  // RIGHT wheel speed (encoder 0)
float speed_e1 = 0.0;  // LEFT wheel speed (encoder 1)

// ============================================
// STAGE 4: PID CONTROL VARIABLES
// ============================================

// PID update timing
unsigned long pid_update_ts = 0;
#define PID_UPDATE_INTERVAL 20  // Update PID every 5ms

// Speed demands (in counts per millisecond)
float demand_speed_left = 0.0;
float demand_speed_right = 0.0;

// PID enable flag
bool use_pid_control = false;  // Start with PID disabled for safety

// PID output values (PWM signals calculated by PID)
float left_pwm_output = 0.0;
float right_pwm_output = 0.0;

// ============================================
// STAGE 5: LINE SENSOR VARIABLES
// ============================================

// Threshold for detecting black line (0.0 = white, 1.0 = black)
// After calibration, readings above this threshold indicate black line
// Start with 0.5 (midpoint), adjust based on testing
#define LINE_THRESHOLD 0.7

// Flag to indicate if line sensors have been calibrated
bool line_sensors_calibrated = false;

// ============================================
// STAGE 9: MAGNETOMETER VARIABLES
// ============================================

// Non-blocking read interval (sensor needs >= 100 ms between reads)
unsigned long mag_read_ts = 0;
#define MAG_READ_INTERVAL 100

// Puck detection threshold on calibrated magnitude.
// Baseline (Earth field only) reads ~1.0 after calibration.
// A magnetic puck within ~80 mm raises the magnitude.
//
// HUMAN CALIBRATION REQUIRED:
//   1. With no puck: confirm magnitude prints ~1.0
//   2. Move puck toward sensor and note the value where
//      you want detection to trigger
//   3. Set PUCK_MAGNITUDE_THRESHOLD to that value
#define PUCK_MAGNITUDE_THRESHOLD 2.0

// Latest magnitude reading (updated by readMagnetometer())
float mag_magnitude = 0.0;

// Whether magnetometer initialised successfully
bool mag_ok = false;

// ============================================
// STAGE 7: KINEMATICS & NAVIGATION VARIABLES
// ============================================

// Pose update timing (should match speed measurement interval)
unsigned long pose_update_ts = 0;
#define POSE_UPDATE_INTERVAL 10  // Update pose every 5ms (matches speed measurement interval)

// Turn behavior variables
float target_angle = 0.0;         // Target orientation (radians)
bool turning = false;              // Is robot currently turning?
#define TURN_THRESHOLD 0.01        // Stop turning when within ±0.01 radians (~0.6°)
#define TURN_GAIN 5.0              // Proportional gain for turning — gives smooth deceleration
                                   // over ~0.1 rad (5.7°) before stopping. Old value of 100
                                   // caused bang-bang: full speed until threshold then instant stop.
#define MAX_TURN_SPEED 0.5         // Maximum turn speed (counts/ms) - adjust based on testing
#define MIN_TURN_SPEED 0.15        // Minimum turn speed to overcome motor dead-band (~PWM 15)

// Relative rotation variables (rotate BY an amount, not TO a heading)
float rotate_target = 0.0;        // Total radians to rotate (positive = CCW)
float rotate_accumulated = 0.0;   // Radians rotated so far
float rotate_prev_theta = 0.0;    // Previous theta for tracking deltas
bool rotating = false;            // Is robot currently doing a relative rotation?

// Travel behavior variables
float target_x = 0.0;              // Target X coordinate (mm)
float target_y = 0.0;              // Target Y coordinate (mm)
float init_x = 0.0;                // Starting X coordinate for travel (mm)
float init_y = 0.0;                // Starting Y coordinate for travel (mm)
float target_init_dist = 0.0;       // Distance to target (mm)
bool traveling = false;            // Is robot currently traveling?
#define TRAVEL_THRESHOLD 10.0      // Stop when within 10mm of target
#define TRAVEL_SPEED 0.55           // Base forward speed (counts/ms) - increased from 0.3 to overcome dead-band (PWM ~15)
#define HEADING_CORRECTION_GAIN 1.0  // Gain for heading correction during travel
#define TRAVEL_DECEL_DISTANCE 10.0 // Start decelerating within 50mm of target (mm)
#define MIN_TRAVEL_SPEED 0.15      // Minimum forward speed to overcome motor dead-band

// Combined travelToXY behavior
enum TravelState {
  TRAVEL_IDLE,       // Not traveling
  TRAVEL_SETTLING,   // Waiting for motors to stop before next phase
  TRAVEL_ROTATING,   // Initial rotation to face target
  TRAVEL_MOVING,     // Moving toward target
  TRAVEL_COMPLETE    // Reached target
};
TravelState travel_state = TRAVEL_IDLE;

// Settling: wait for motors to physically stop between movement phases
#define SETTLE_TIME_MS 50            // Minimum settling wait (ms)
#define SETTLE_SPEED_THRESHOLD 0.02  // Speed below this = stopped (counts/ms)
unsigned long settle_start_ms = 0;
TravelState settle_next_state = TRAVEL_ROTATING;  // State to enter after settling

// ============================================
// STAGE 6: FSM STATES & TIMER
// ============================================

// Robot behaviour states
enum RobotState {
  STATE_CALIBRATING_MAGNETOMETER, // Calibrate magnetometer (3 rotations)
  STATE_TRAVEL_TO_WAYPOINT,       // Navigating to the current puck waypoint
  STATE_AT_WAYPOINT,              // Arrived – brief pause, check for puck
  STATE_TRAVERSE_PUCKED_WAYPOINT, // Manoeuvring around detected puck
  STATE_RETURN_HOME,              // Navigating back to start (0,0), pushing puck if present
  STATE_STOPPED,
  STATE_WAITING,                   // Arrived home, run complete
};
RobotState robot_state = STATE_CALIBRATING_MAGNETOMETER;

unsigned long waiting_ts = 0;
#define WAITING_DURATION_MS 4000  // Time to wait at home before restarting search (optional)

// ============================================
// STAGE 10: PUCK TRAVERSE SUB-PHASES
// ============================================
// When a puck is detected at a waypoint, the robot reverses,
// navigates 3 waypoints to get behind the puck (on the
// puck-to-home line), turns to face home, then pushes.
enum TraversePhase {
  TRAVERSE_REVERSE,     // Backing away from detected puck
  TRAVERSE_WP1,         // Navigating to bypass waypoint (or direct to push pos)
  TRAVERSE_WP2          // Navigating to push position behind puck (far quadrant only)
};
TraversePhase traverse_phase;

// Whether the push position (rF) is in the "far" quadrant
// relative to the robot's approach, requiring an extra bypass WP.
// Set during TRAVERSE_REVERSE, read during TRAVERSE_WP1.
bool in_far_quadrant = false;

// ============================================
// STAGE 9: MAGNETOMETER CALIBRATION STATE
// ============================================
bool mag_calibration_in_progress = false;
unsigned long mag_calibration_last_read = 0;
unsigned long post_calibration_grace_end = 0;  // Disable puck detection until sensor settles
#define POST_CALIBRATION_GRACE_MS 2000         // 2-second grace period after calibration

// Computed traverse waypoints (max 2: bypass + push position)
float traverse_wp_x[2];
float traverse_wp_y[2];

// Lateral/forward offsets for WP1 and WP2 (mm).
// WP1 = (puck_x - LATERAL, puck_y - FORWARD)
// WP2 = (puck_x + LATERAL, puck_y - FORWARD)
// Tune these on the physical robot.
#define TRAVERSE_OFFSET 170.0

// How far behind the puck (away from home) to place WP3 (mm)

// (Reverse distance is controlled by REVERSE_DISTANCE above)

// 4-minute countdown timer.
// Started in setup() AFTER line-sensor calibration so the 3 s
// calibration spin does not eat into the 4-minute run.
#define TIMER_DURATION_MS 240000UL   // 4 minutes in ms
unsigned long timer_start_ms = 0;    // Set in setup()

// Serial countdown print (updates once per second)
unsigned long countdown_print_ts = 0;

// Shared timestamp for timed manoeuvres (reverse / turn)
unsigned long maneuver_ts = 0;

// --- Reversing parameters ---
#define REVERSE_SPEED -0.3        // Reverse speed (counts/ms, negative)
#define REVERSE_DISTANCE 100.0     // Default reverse distance (mm) - tune on robot
float reverse_target_dist = 0.0;  // Distance to reverse (mm)
float reverse_start_x = 0.0;     // Pose X when reverse began
float reverse_start_y = 0.0;     // Pose Y when reverse began
bool reversing_dist = false;      // Is a distance-based reverse in progress?

// --- Turning parameters ---
#define MIN_TURN_DURATION 300   // Shortest possible turn (ms)
#define MAX_TURN_DURATION 1000  // Longest  possible turn (ms)
#define TURN_SPEED 0.4          // Wheel speed during turn (counts/ms)
unsigned long turn_duration = 0;  // Chosen randomly each turn
int turn_direction = 1;           // +1 = left, -1 = right

// ============================================
// STAGE 8: WAYPOINT NAVIGATION
// ============================================
// Number of puck locations extracted from Map.svg
#define NUM_WAYPOINTS 6

// Puck locations in the robot coordinate frame (mm).
// Origin = start-area centre; robot faces +X at launch.
// Coordinates extracted from Map.svg by chaining nested
// group translate() transforms.
float waypoint_x[NUM_WAYPOINTS] = {  60.6, 260.7, 429.0, 408.8, 354.3, 156.7 };
float waypoint_y[NUM_WAYPOINTS] = { 262.9, 211.4, 267.7, 113.9, -10.9,  83.9 };

// Visit order: indices into waypoint arrays.
// Sweep order roughly minimises total travel distance.
int waypoint_order[NUM_WAYPOINTS] = { 0, 1, 2, 3, 4, 5 };

// Index into waypoint_order for the waypoint currently being visited
int current_waypoint_idx = 0;

// How long (ms) to pause at each waypoint before moving on.
// Leaves time for future puck-detection (Stage 9).
#define WAYPOINT_PAUSE_MS 50

// The setup() function runs only once when the
// robot is powered up (either by plugging in
// the USB cable, or activating the motor power.
// Use this function to do "once only" setup 
// and configuration of your robot.
void setup() {
  // Setup up the buzzer as an output
  pinMode( BUZZER_PIN, OUTPUT );

  // Setup button A as input with internal pull-up resistor
  // Button reads LOW when pressed, HIGH when released
  pinMode( BUTTON_A_PIN, INPUT_PULLUP );

  // Setup motors.  This is calling a function
  // from within the Motors_c class. You can 
  // review this inside Motors.h
  // Complete by working through Labsheet 1.
  motors.initialise();

  // Setup the line sensors.  This is calling a 
  // function from within the LineSensors_c 
  // class. You can review this inside 
  // LineSensors.h.  
  // Complete by working through Labsheet 2.
  line_sensors.initialiseForADC();

  // These two functions are in encoders.h and
  // they activate the encoder sensors to 
  // measure the wheel rotation.  You do not 
  // need to change or update these.
  // These are used in Labsheet 4.
  // Encoder counts are counted automatically.
  setupEncoder0();
  setupEncoder1();

  // Setup the pose (kinematics). This is calling 
  // a function within the Kinematics_c class.
  // You can review this within Kinematics.h
  // This is used in Labsheet 4.
  pose.initialise(0, 0, 0);

  // Activates the Serial port, and the delay
  // is used to wait for the connection to be
  // established.
  Serial.begin(9600);
  delay(2000);
  Serial.println(" *** READY *** ");

  // ============================================
  // STAGE 3: Initialize speed measurement
  // ============================================
  // Capture initial encoder counts
  prev_count_e0 = count_e0;
  prev_count_e1 = count_e1;

  // Initialize speeds to zero (robot starts stationary)
  speed_e0 = 0.0;
  speed_e1 = 0.0;

  // Capture starting timestamp
  speed_measurement_ts = millis();

  // ============================================
  // STAGE 4: Initialize PID Controllers
  // ============================================

  // Starting gains for PID tuning
  // BEGIN WITH P-ONLY CONTROL: Kp = 100, Ki = 0.0, Kd = 0.0
  // These are initial guesses and MUST be tuned physically
  // NOTE: Kp must be large because speed is in counts/ms (small values)
  //       but PWM needs to be 20-255 (large values)
  float initial_Kp = 48.0;  // Start with proportional control only
  float initial_Ki = 0.5;    // Integral term disabled initially
  float initial_Kd = 0.0;    // Derivative term disabled initially

  // Initialize left motor PID
  left_pid.initialise(initial_Kp, initial_Ki, initial_Kd);

  // Initialize right motor PID
  right_pid.initialise(initial_Kp, initial_Ki, initial_Kd);

  // Initialize demand speeds to zero (stopped)
  demand_speed_left = 0.0;
  demand_speed_right = 0.0;

  // PID starts disabled for safety
  // Enable by setting use_pid_control = true in your code
  use_pid_control = false;

  // Capture starting timestamp for PID updates
  pid_update_ts = millis();

  // ============================================
  // STAGE 6: Initialize OLED Display Timer
  // ============================================
  // Start the timer BEFORE magnetometer calibration so the 4-minute
  // countdown doesn't include calibration time.
  display.reset();          // Initialize OLED display (with USB interrupt protection)
  display.setMaxMinutes(4); // Set timer for 4 minutes
  display.startStopwatch(); // Start the countdown
  timer_start_ms = millis(); // Record timer start for 4-minute check in loop()
  // ============================================
  // STAGE 5: Line Sensor Calibration
  // ============================================
  // The robot will rotate on the spot over black and white surfaces
  // to learn the min/max sensor values for calibration
  //
  // IMPORTANT: Place robot on the calibration disk (alternating
  // black/white pattern at start position) before powering on!
  //
  // Uncomment the following line to enable calibration at startup:
  //calibrateLineSensors();

  //Serial.println("Stage 5: Line sensors initialized");

  // ============================================
  // STAGE 9: Initialize Magnetometer (calibration deferred to FSM)
  // ============================================
  // Initialize the magnetometer hardware.
  // Actual calibration will happen in STATE_CALIBRATING_MAGNETOMETER.
  mag_ok = magnetometer.initialise();
  if ( mag_ok ) {
    Serial.println("Stage 9: Magnetometer found on I2C bus");
    Serial.println("  Calibration will occur in STATE_CALIBRATING_MAGNETOMETER");
  } else {
    Serial.println("WARNING: Magnetometer NOT found! Puck detection disabled.");
  }
  // ============================================
  // WAIT FOR BUTTON A PRESS TO START
  // ============================================
  // All hardware is now initialised. Wait here
  // so you can position the robot on the
  // calibration disk before anything moves.
  // Serial.println();
  // Serial.println("=================================");
  // Serial.println("  Press BUTTON A to start robot");
  // Serial.println("=================================");

  // while (digitalRead(BUTTON_A_PIN) != LOW) {
  //   // Wait for button press
  // }
  // // Debounce: wait for button release
  // delay(50);
  // while (digitalRead(BUTTON_A_PIN) == LOW) {}
  // delay(50);

  // tone(BUZZER_PIN, 440, 100);  // Short beep to confirm start
  // Serial.println("*** ROBOT STARTED ***");


  // Re-initialise pose AFTER calibration spins so rotation
  // does not corrupt the starting odometry.
  //pose.initialise(0, 0, 0);

  // ============================================
  // STAGE 7: Initialize Kinematics
  // ============================================
  // Capture starting timestamp for pose updates
  pose_update_ts = millis();

  // ============================================
  // STAGE 6: Start FSM & Timer
  // ============================================
  // Seed RNG (used for random turn durations / directions)
  randomSeed(analogRead(A0) + millis());

  // Enable PID closed-loop speed control for all driving
  use_pid_control = true;

  // FSM will start in STATE_CALIBRATING_MAGNETOMETER and proceed to
  // STATE_TRAVEL_TO_WAYPOINT once calibration is complete.
  // Don't call travelToXY() here — the FSM handles it.
}

// ============================================
// STAGE 5: LINE SENSOR CALIBRATION FUNCTION
// ============================================
// This function rotates the robot on the spot while collecting
// sensor readings to determine min/max values for each sensor.
// Call this once at startup with robot on calibration disk.
void calibrateLineSensors() {

  Serial.println("Starting line sensor calibration...");
  Serial.println("Robot will rotate for 3 seconds.");

  // Reset calibration to prepare for new values
  line_sensors.resetCalibration();

  // Start rotating on the spot (left wheel backward, right wheel forward)
  // Use low speed for better calibration
  motors.setPWM(-40, 40);

  // Calibration duration in milliseconds
  unsigned long calibration_duration = 3000;
  unsigned long start_time = millis();

  // Collect readings while rotating
  while (millis() - start_time < calibration_duration) {
    // Update calibration with new readings
    line_sensors.updateCalibration();

    // Small delay between readings
    delay(10);
  }

  // Stop motors
  motors.setPWM(0, 0);

  // Calculate final calibration values (scaling = range)
  line_sensors.finalizeCalibration();

  // Mark as calibrated
  line_sensors_calibrated = true;

  // Report calibration results
  Serial.println("Calibration complete!");
  Serial.println("Sensor calibration values:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(": min=");
    Serial.print(line_sensors.minimum[i]);
    Serial.print(", max=");
    Serial.print(line_sensors.maximum[i]);
    Serial.print(", range=");
    Serial.println(line_sensors.scaling[i]);
  }

  // Short delay before continuing
  delay(500);
}


// ============================================
// STAGE 9: MAGNETOMETER CALIBRATION FUNCTION
// ============================================
// Rotates the robot exactly 3 full revolutions using
// setRotation/checkRotation (PID-controlled), collecting
// magnetometer min/max per axis throughout.  The robot
// finishes facing the same direction it started (+X).
//
// Runs in setup() as a blocking loop that manually pumps
// the system update functions (speeds, PID, pose).
//
// Place the robot on a flat surface AWAY from metal objects
// (table legs, laptops, phones) before running.
void calibrateMagnetometer() {

  Serial.println("Starting magnetometer calibration...");
  Serial.println("Robot will rotate 3 full turns (PID-controlled).");

  magnetometer.resetCalibration();

  // Temporarily enable PID for controlled rotation
  bool pid_was_enabled = use_pid_control;
  use_pid_control = true;
  left_pid.reset();
  right_pid.reset();

  // 3 full CCW revolutions = 6*PI radians
  setRotation(3.0 * 2.0 * PI);

  unsigned long last_mag_read = 0;

  // Blocking loop: pump system updates + rotation + mag reads
  while ( !checkRotation() ) {
    measureSpeeds();
    updatePID();
    updatePose();

    // Read magnetometer every 100 ms (sensor timing limit)
    if ( millis() - last_mag_read >= 100 ) {
      magnetometer.updateCalibration();
      last_mag_read = millis();
    }
  }

  // Ensure motors are stopped
  stopWithPID();

  // Restore previous PID state
  use_pid_control = pid_was_enabled;

  // Compute offset and scaling from captured min/max
  magnetometer.finalizeCalibration();

  // Report calibration results
  Serial.println("Magnetometer calibration complete!");
  for ( int i = 0; i < MAG_AXIS; i++ ) {
    Serial.print("  Axis ");
    Serial.print(i);
    Serial.print(": min=");
    Serial.print(magnetometer.minimum[i], 0);
    Serial.print(", max=");
    Serial.print(magnetometer.maximum[i], 0);
    Serial.print(", offset=");
    Serial.print(magnetometer.offset[i], 1);
    Serial.print(", scaling=");
    Serial.println(magnetometer.scaling[i], 1);
  }

  delay(500);
}

// ============================================
// STAGE 9: NON-BLOCKING MAGNETOMETER READ
// ============================================
// Call from loop(). Respects the 100 ms minimum interval
// between I2C reads.  Updates mag_magnitude with the
// latest calibrated vector magnitude.
void readMagnetometer() {
  if ( !mag_ok ) return;  // Sensor failed to init

  unsigned long now = millis();
  if ( now - mag_read_ts >= MAG_READ_INTERVAL ) {
    magnetometer.getReadings();
    mag_magnitude = magnetometer.getMagnitude();
    mag_read_ts = now;

    // TEMPORARY: print magnitude for threshold calibration
    // Remove this once PUCK_MAGNITUDE_THRESHOLD is set
    Serial.print("MAG:");
    Serial.println(mag_magnitude, 3);
  }
}

// ============================================
// STAGE 9: PUCK DETECTION WRAPPER
// ============================================
// Returns true if the latest magnitude reading exceeds
// PUCK_MAGNITUDE_THRESHOLD.  Safe to call every loop
// iteration; it uses the cached mag_magnitude updated
// by readMagnetometer().
// Returns false during the post-calibration grace period
// to allow magnetometer to settle.
bool detectPuck() {
  // Disable detection during grace period after calibration
  if (millis() < post_calibration_grace_end) {
    return false;
  }
  return ( mag_magnitude > PUCK_MAGNITUDE_THRESHOLD );
}


// Function to measure and calculate wheel speeds
// This is non-blocking and should be called regularly from loop()
void measureSpeeds() {

  // Check if it's time to measure speed
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - speed_measurement_ts;

  if (elapsed_time >= SPEED_MEASUREMENT_INTERVAL) {

    // Atomic read: disable interrupts while copying volatile
    // encoder counts (4-byte long is not atomic on 8-bit AVR)
    long local_e0, local_e1;
    noInterrupts();
    local_e0 = count_e0;
    local_e1 = count_e1;
    interrupts();

    // Calculate change in encoder counts since last measurement
    long delta_e0 = local_e0 - prev_count_e0;
    long delta_e1 = local_e1 - prev_count_e1;

    // Save current counts for next measurement
    prev_count_e0 = local_e0;
    prev_count_e1 = local_e1;

    // Calculate speed as counts per millisecond
    // Typecast to float for precision
    // Speed = change in counts / change in time
    speed_e0 = (float)delta_e0 / (float)elapsed_time;
    speed_e1 = (float)delta_e1 / (float)elapsed_time;

    // Update timestamp for next measurement
    speed_measurement_ts = current_time;

    // Optional: Print speeds for debugging (comment out if not needed)
    // Serial.print("L:");
    // Serial.print(speed_e0, 4);
    // Serial.print(",R:");
    // Serial.println(speed_e1, 4);
  }
}

// ============================================
// STAGE 4: PID CONTROL FUNCTION
// ============================================

/*
 * Update PID controllers and apply motor commands
 *
 * This function should be called regularly from loop()
 * It runs every 50ms to give PID controllers time to respond
 *
 * When use_pid_control is true:
 *   - PID controllers calculate PWM based on speed error
 *   - Motors are controlled to achieve demand speeds
 *
 * When use_pid_control is false:
 *   - Direct PWM control (Stage 2 behavior)
 *   - PID controllers are reset to prevent integral windup
 */
void updatePID() {

  // Check if it's time to update PID (every 50ms)
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - pid_update_ts;

  if (elapsed_time >= PID_UPDATE_INTERVAL) {

    if (use_pid_control) {
      // ============================================
      // PID CLOSED-LOOP CONTROL ENABLED
      // ============================================

      // Update left motor PID
      // Inputs: demand speed, measured speed
      // Output: PWM value to apply to motor
      // NOTE: Encoder mapping from Encoders.h:
      //   count_e0 / speed_e0 = RIGHT wheel
      //   count_e1 / speed_e1 = LEFT wheel
      if (demand_speed_left == 0.0) {
        left_pwm_output = 0.0;
        left_pid.reset();  // Clear stale integral to prevent jump on resume
      } else {
        left_pwm_output = left_pid.update(demand_speed_left, speed_e1);
      }
      // Update right motor PID
      if (demand_speed_right == 0.0) {
        right_pwm_output = 0.0;
        right_pid.reset();  // Clear stale integral to prevent jump on resume
      } else {
        right_pwm_output = right_pid.update(demand_speed_right, speed_e0);
      }
      // Apply PID outputs to motors
      motors.setPWM(left_pwm_output, right_pwm_output);

      // Debug output for tuning
      // NOTE: speed_e1 = LEFT, speed_e0 = RIGHT (from Encoders.h)
      //Serial.print("DemL:");
      //Serial.print(demand_speed_left, 4);
      //Serial.print(",MeasL:");
      //Serial.print(speed_e1, 4);           // Corrected: e1 is left
      //Serial.print(",PWML:");
      //Serial.print(left_pwm_output, 1);
      //Serial.print(" | DemR:");
      //Serial.print(demand_speed_right, 4);
      //Serial.print(",MeasR:");
      //Serial.print(speed_e0, 4);           // Corrected: e0 is right
      //Serial.print(",PWMR:");
      //Serial.println(right_pwm_output, 1);

    } else {
      // ============================================
      // PID DISABLED - Direct PWM control
      // ============================================

      // Reset PID controllers to prevent integral windup
      // This ensures clean startup when PID is re-enabled
      left_pid.reset();
      right_pid.reset();

      // Direct PWM control is handled elsewhere in code
      // (e.g., motor test states or manual commands)
    }

    // Update timestamp for next PID cycle
    pid_update_ts = current_time;
  }
}

/*
 * Set target speeds for both motors
 *
 * @param left_speed: Target speed for left wheel (counts/ms)
 * @param right_speed: Target speed for right wheel (counts/ms)
 *
 * Note: This only has effect when use_pid_control = true
 */
void setDemandSpeeds(float left_speed, float right_speed) {
  demand_speed_left = left_speed;
  demand_speed_right = 0.88*right_speed;
}

/*
 * Convenience function: Set same speed for both motors (straight line)
 *
 * @param speed: Target speed for both wheels (counts/ms)
 */
void setDemandSpeed(float speed) {
  setDemandSpeeds(speed, speed);
}

/*
 * Stop motors smoothly using PID
 * Sets demand speeds to zero
 */
void stopWithPID() {
  setDemandSpeeds(0.0, 0.0);
  motors.setPWM(0, 0);  // Zero PWM directly for immediate stop
  left_pid.reset();
  right_pid.reset();
}

// ============================================
// STAGE 5: LINE SENSOR HELPER FUNCTIONS
// ============================================

/*
 * Check if any line sensor detects a black line
 * Uses calibrated readings if calibration was performed
 *
 * @return true if any sensor detects black line, false otherwise
 */
bool isOnLine() {
  if (line_sensors_calibrated) {
    // Use calibrated isOnLine with threshold
    return line_sensors.isOnLine(LINE_THRESHOLD);
  } else {
    // Without calibration, use raw readings with typical threshold
    // Raw ADC values: ~600-700 for white, ~900-1000 for black (typical)
    line_sensors.readSensorsADC();
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (line_sensors.readings[i] > 800) {  // Typical threshold for uncalibrated
        return true;
      }
    }
    return false;
  }
}

/*
 * Check if a specific sensor detects the line
 *
 * @param sensor_index: 0=DN1(left), 1=DN2, 2=DN3(center), 3=DN4, 4=DN5(right)
 * @return true if specified sensor is on line
 */
bool isSensorOnLine(int sensor_index) {
  if (line_sensors_calibrated) {
    return line_sensors.isSensorOnLine(sensor_index, LINE_THRESHOLD);
  } else {
    line_sensors.readSensorsADC();
    if (sensor_index >= 0 && sensor_index < NUM_SENSORS) {
      return line_sensors.readings[sensor_index] > 800;
    }
    return false;
  }
}

/*
 * Get pattern of which sensors are on the line
 * Useful for determining line position relative to robot
 *
 * @return bitmask: bit 0=DN1(left), bit 1=DN2, bit 2=DN3(center), etc.
 *         Example: 0b00100 (=4) means only center sensor on line
 *         Example: 0b00001 (=1) means only left sensor on line
 *         Example: 0b10000 (=16) means only right sensor on line
 */
byte getLineSensorPattern() {
  if (line_sensors_calibrated) {
    return line_sensors.getLineSensorPattern(LINE_THRESHOLD);
  } else {
    line_sensors.readSensorsADC();
    byte pattern = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (line_sensors.readings[i] > 800) {
        pattern |= (1 << i);
      }
    }
    return pattern;
  }
}

/*
 * Print line sensor readings for debugging
 * Prints calibrated values if calibrated, raw values otherwise
 */
void printLineSensorReadings() {
  if (line_sensors_calibrated) {
    line_sensors.calcCalibratedADC();
    Serial.print("Cal: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(line_sensors.calibrated[i], 2);
      if (i < NUM_SENSORS - 1) Serial.print(", ");
    }
  } else {
    line_sensors.readSensorsADC();
    Serial.print("Raw: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(line_sensors.readings[i], 0);
      if (i < NUM_SENSORS - 1) Serial.print(", ");
    }
  }
  Serial.println();
}

// ============================================
// STAGE 7: KINEMATICS UPDATE FUNCTION
// ============================================

/*
 * Update robot pose (x, y, theta) using encoder counts
 * MUST be called regularly at fixed interval (20ms recommended)
 * Call this from loop() after measureSpeeds()
 */
void updatePose() {
  unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - pose_update_ts;

  if (elapsed_time >= POSE_UPDATE_INTERVAL) {
    pose.update();  // Update kinematics
    pose_update_ts = current_time;
  }
}

// ============================================
// STAGE 7: NAVIGATION HELPER FUNCTIONS
// ============================================

/*
 * Calculate the smallest angular difference between two angles
 * Handles wraparound at 0/2π boundary
 *
 * @param target_angle: Desired angle (radians)
 * @param current_angle: Current angle (radians)
 * @return: Smallest difference in radians, range [-π, π]
 *          Positive = turn left, Negative = turn right
 */
float angleDiff(float target_angle, float current_angle) {
  // Calculate raw difference
  float diff = target_angle - current_angle;

  // Normalize to [-π, π] using atan2 trick
  // This handles the 0°/360° wraparound elegantly
  diff = atan2(sin(diff), cos(diff));

  return diff;
}

/*
 * Calculate angle from current position to target (x,y)
 * Uses atan2 to handle all quadrants correctly
 *
 * @param target_x: Target X coordinate (mm)
 * @param target_y: Target Y coordinate (mm)
 * @return: Angle to target in radians
 */
float angleToPoint(float target_x, float target_y) {
  float dx = target_x - pose.x;
  float dy = target_y - pose.y;
  return atan2(dy, dx);
}

/*
 * Calculate distance from current position to target (x,y)
 *
 * @param target_x: Target X coordinate (mm)
 * @param target_y: Target Y coordinate (mm)
 * @return: Distance to target in mm
 */
float distanceToPoint(float target_x, float target_y) {
  float dx = target_x - pose.x;
  float dy = target_y - pose.y;
  return sqrt(dx*dx + dy*dy);
}

// ============================================
// STAGE 7: TURN BEHAVIOR
// ============================================

/*
 * Initialize a turn to a specific angle
 * Call this ONCE when you want to start a turn
 *
 * @param angle: Target orientation in radians
 */
void setTurn(float angle) {
  target_angle = angle;
  turning = true;
}

/*
 * Non-blocking turn update function
 * Call this repeatedly from loop() while turning
 *
 * @return: true if turn complete, false if still turning
 */
bool checkTurn() {
  if (!turning) {
    return true;  // Not turning, return complete
  }

  // Calculate angular difference (smallest angle)
  float angle_error = angleDiff(target_angle, pose.theta);

  // Check if turn is complete
  if (abs(angle_error) < TURN_THRESHOLD) {
    stopWithPID();
    turning = false;
    return true;  // Turn complete
  }

  // Calculate turn speed proportional to error
  float turn_speed = angle_error * TURN_GAIN;
  // Constrain turn speed to maximum allowed value
  if (turn_speed > MAX_TURN_SPEED) turn_speed = MAX_TURN_SPEED;
  if (turn_speed < -MAX_TURN_SPEED) turn_speed = -MAX_TURN_SPEED;
  // Floor at minimum speed to overcome motor dead-band
  if (fabs(turn_speed) < MIN_TURN_SPEED) {
    turn_speed = (angle_error > 0) ? MIN_TURN_SPEED : -MIN_TURN_SPEED;
  }

  // Apply turn speeds (opposite directions for rotation on spot)
  if (use_pid_control) {
    // Using PID speed control
    // Left wheel: positive speed turns left
    // Right wheel: negative speed turns right
    setDemandSpeeds(-turn_speed, turn_speed);
  } else {
    // Direct PWM control
    float turn_pwm = turn_speed;
    // Constrain to reasonable PWM range
    if (turn_pwm > 100) turn_pwm = 100;
    if (turn_pwm < -100) turn_pwm = -100;
    motors.setPWM(turn_pwm, -turn_pwm);
  }

  return false;  // Still turning
}

// ============================================
// STAGE 7: RELATIVE ROTATION BEHAVIOR
// ============================================

/*
 * Start a rotation BY a given number of radians (relative).
 * Unlike setTurn() which targets an absolute heading, this
 * accumulates angular change so it can handle multi-revolution
 * rotations (e.g. 3 full turns = 6*PI).
 *
 * @param radians: Amount to rotate. Positive = CCW, Negative = CW.
 */
void setRotation(float radians) {
  rotate_target = radians;
  rotate_accumulated = 0.0;
  rotate_prev_theta = pose.theta;
  rotating = true;
}

/*
 * Non-blocking rotation update function.
 * Call this repeatedly from loop() while rotating.
 * Uses the same TURN_GAIN, MAX_TURN_SPEED and TURN_THRESHOLD
 * as checkTurn() for consistent behaviour.
 *
 * @return: true if rotation complete, false if still rotating
 */
bool checkRotation() {
  if (!rotating) return true;

  // Accumulate the small angular change since last call.
  // angleDiff handles the ±PI wraparound per step so the
  // running total can exceed 2*PI for multi-revolution spins.
  float delta = angleDiff(pose.theta, rotate_prev_theta);
  rotate_accumulated += delta;
  rotate_prev_theta = pose.theta;

  // How much rotation remains
  float remaining = rotate_target - rotate_accumulated;

  // Check if rotation is complete
  if (abs(remaining) < TURN_THRESHOLD) {
    stopWithPID();
    rotating = false;
    return true;
  }

  // Proportional speed control on remaining angle
  float turn_speed = remaining * TURN_GAIN;
  if (turn_speed > MAX_TURN_SPEED) turn_speed = MAX_TURN_SPEED;
  if (turn_speed < -MAX_TURN_SPEED) turn_speed = -MAX_TURN_SPEED;
  // Floor at minimum speed to overcome motor dead-band
  if (fabs(turn_speed) < MIN_TURN_SPEED) {
    turn_speed = (remaining > 0) ? MIN_TURN_SPEED : -MIN_TURN_SPEED;
  }

  if (use_pid_control) {
    setDemandSpeeds(-turn_speed, turn_speed);
  } else {
    float turn_pwm = turn_speed;
    if (turn_pwm > 100) turn_pwm = 100;
    if (turn_pwm < -100) turn_pwm = -100;
    motors.setPWM(turn_pwm, -turn_pwm);
  }

  return false;
}

// ============================================
// STAGE 7: REVERSE BEHAVIOR
// ============================================

/*
 * Start a straight-line reverse by a given distance.
 * Non-blocking: call checkReverse() each loop iteration.
 * Uses odometry (pose) to measure distance travelled.
 *
 * @param distance_mm: How far to reverse (positive value, mm).
 *                     Defaults to REVERSE_DISTANCE if 0 is passed.
 */
void setReverse(float distance_mm) {
  reverse_target_dist = (distance_mm > 0) ? distance_mm : REVERSE_DISTANCE;
  reverse_start_x = pose.x;
  reverse_start_y = pose.y;
  reversing_dist = true;
}

/*
 * Non-blocking reverse update function.
 * Call this repeatedly from loop() while reversing.
 *
 * @return: true if reverse complete, false if still reversing
 */
bool checkReverse() {
  if (!reversing_dist) return true;

  // Calculate distance from starting position
  float dx = pose.x - reverse_start_x;
  float dy = pose.y - reverse_start_y;
  float dist_traveled = sqrt(dx * dx + dy * dy);

  // Check if target distance reached
  if (dist_traveled >= reverse_target_dist) {
    stopWithPID();
    reversing_dist = false;
    return true;
  }

  // Drive backward at constant reverse speed
  if (use_pid_control) {
    setDemandSpeeds(REVERSE_SPEED, REVERSE_SPEED);
  } else {
    motors.setPWM(-30, -30);
  }

  return false;
}

// ============================================
// STAGE 7: TRAVEL BEHAVIOR
// ============================================

/*
 * Initialize travel to a specific (x,y) coordinate
 * Robot will drive straight, making small heading corrections
 * Call this ONCE when you want to start traveling
 *
 * @param x: Target X coordinate (mm)
 * @param y: Target Y coordinate (mm)
 */
void setTravel(float x, float y) {
  target_x = x;
  target_y = y;
  init_x = pose.x;  // Capture starting position for distance tracking
  init_y = pose.y;
  target_init_dist = distanceToPoint(target_x, target_y);
  traveling = true;
}

/*
 * Non-blocking travel update function
 * Call this repeatedly from loop() while traveling
 * Robot moves toward target while correcting heading
 *
 * @return: true if travel complete, false if still traveling
 */
bool checkTravel() {
  if (!traveling) return true;  // Not traveling, return complete

  // Calculate distance to target
  float distance = distanceToPoint(target_x, target_y);
  float distance_traveled = distanceToPoint(init_x, init_y);
  // Check if travel is complete
  if (distance < TRAVEL_THRESHOLD) {
    stopWithPID();
    traveling = false;
    return true;  // Travel complete
  }

  // Calculate desired heading to target
  float desired_heading = angleToPoint(target_x, target_y);

  // Calculate heading error
  float heading_error = angleDiff(desired_heading, pose.theta);

  // Calculate base forward speed with deceleration near target
  float forward_speed = TRAVEL_SPEED;
  if (distance < TRAVEL_DECEL_DISTANCE) {
    forward_speed = TRAVEL_SPEED * (distance / TRAVEL_DECEL_DISTANCE);
    if (forward_speed < MIN_TRAVEL_SPEED) forward_speed = MIN_TRAVEL_SPEED;
  }

  // Calculate heading correction
  float heading_correction = heading_error * HEADING_CORRECTION_GAIN;

  // Apply differential speeds for heading correction
  float left_speed = forward_speed - heading_correction;
  float right_speed = forward_speed + heading_correction;

  if (use_pid_control) {
    setDemandSpeeds(left_speed, right_speed);
  } else {
    // Direct PWM control
    float left_pwm = left_speed * 100;  // Scale to PWM
    float right_pwm = right_speed * 100;
    // Constrain PWM
    if (left_pwm > 100) left_pwm = 100;
    if (left_pwm < -100) left_pwm = -100;
    if (right_pwm > 100) right_pwm = 100;
    if (right_pwm < -100) right_pwm = -100;
    motors.setPWM(left_pwm, right_pwm);
  }

  return false;  // Still traveling
}

// ============================================
// STAGE 7: COMBINED TRAVEL-TO-XY BEHAVIOR
// ============================================

/*
 * High-level navigation function
 * Rotates to face target, then travels to it
 * This is the recommended function for waypoint navigation
 *
 * Call this ONCE to start navigation:
 *   travelToXY(x, y);
 *
 * Then call updateTravelToXY() repeatedly from loop():
 *   updateTravelToXY();
 *
 * @param x: Target X coordinate (mm)
 * @param y: Target Y coordinate (mm)
 */
void travelToXY(float x, float y) {
  target_x = x;
  target_y = y;

  // Stop motors and reset PIDs before settling
  stopWithPID();
  settle_start_ms = millis();
  settle_next_state = TRAVEL_ROTATING;
  travel_state = TRAVEL_SETTLING;
}

/*
 * Non-blocking update for travelToXY behavior
 * Call this repeatedly from loop()
 *
 * @return: true if navigation complete, false otherwise
 */
bool updateTravelToXY() {
  switch (travel_state) {
    case TRAVEL_IDLE:
      // Not navigating
      return true;

    case TRAVEL_SETTLING: {
      // Wait for motors to physically stop before starting the next phase.
      // Requires both a minimum time AND near-zero wheel speeds.
      bool time_ok = (millis() - settle_start_ms >= SETTLE_TIME_MS);
      bool speed_ok = (fabs(speed_e0) < SETTLE_SPEED_THRESHOLD &&
                       fabs(speed_e1) < SETTLE_SPEED_THRESHOLD);
      if (time_ok && speed_ok) {
        left_pid.reset();   // Fresh PID state for upcoming phase
        right_pid.reset();
        travel_state = settle_next_state;
      }
      return false;
    }

    case TRAVEL_ROTATING: {

      // Calculate angle to target
      float desired_angle = angleToPoint(target_x, target_y);

      // If not yet started turning, initialize turn
      if (!turning) {
        setTurn(desired_angle);
      }

      // Update turn
      if (checkTurn()) {
        // Turn complete — settle before traveling
        stopWithPID();
        settle_start_ms = millis();
        settle_next_state = TRAVEL_MOVING;
        travel_state = TRAVEL_SETTLING;
      }
      return false;
    }

    case TRAVEL_MOVING:

      // If not yet started traveling, initialize travel
      if (!traveling) {
        setTravel(target_x, target_y);
      }

      // Update travel
      if (checkTravel()) {
        // Travel complete
        travel_state = TRAVEL_COMPLETE;
      }
      return false;

    case TRAVEL_COMPLETE:
      travel_state = TRAVEL_IDLE;
      return true;
  }

  return true;
}

// put your main code here, to run repeatedly:
void loop() {

  // ============================================
  // CONTINUOUS SYSTEM UPDATES (every iteration)
  // ============================================
  measureSpeeds();      // Stage 3 – encoder speeds   (20 ms interval)
  updatePID();          // Stage 4 – closed-loop motor control
  updatePose();         // Stage 7 – dead-reckoning odometry (20 ms interval)
  readMagnetometer();   // Stage 9 – puck detection   (100 ms interval)

  // ============================================
  // STAGE 6: 4-MINUTE TIMER CHECK
  // ============================================
  // If time is up and we're not already heading home or stopped,
  // abandon current task and return to start immediately.
  if (millis() - timer_start_ms >= TIMER_DURATION_MS ) {
    Serial.println("*** 4-MINUTE TIMER EXPIRED — STOPPING ***");
    stopWithPID();
    robot_state = STATE_STOPPED;
  }

  // ============================================
  // STAGE 6: MAIN FINITE STATE MACHINE
  // ============================================
  switch (robot_state) {

    // --------------------------------------------------
    // CALIBRATING_MAGNETOMETER – spin 3 rotations collecting min/max
    // --------------------------------------------------
    case STATE_CALIBRATING_MAGNETOMETER:
      if (!mag_ok) {
        // Magnetometer not available, skip calibration
        Serial.println("Magnetometer not available, skipping calibration");
        robot_state = STATE_TRAVEL_TO_WAYPOINT;
        travelToXY(waypoint_x[waypoint_order[0]], waypoint_y[waypoint_order[0]]);
      } else if (!mag_calibration_in_progress) {
        // Start calibration: reset and begin 3-rotation spin
        magnetometer.resetCalibration();

        // Setup PID for controlled rotation
        bool pid_was_enabled = use_pid_control;
        use_pid_control = true;
        left_pid.reset();
        right_pid.reset();

        // Set target: 3 full CCW revolutions = 6*PI radians
        setRotation(3.0 * 2.0 * PI);
        mag_calibration_in_progress = true;
        mag_calibration_last_read = 0;
      } else {
        // Calibration in progress: pump system updates + rotation + mag reads
        measureSpeeds();
        updatePID();
        updatePose();

        // Read magnetometer every 100ms (sensor timing limit)
        if (millis() - mag_calibration_last_read >= 100) {
          magnetometer.updateCalibration();
          mag_calibration_last_read = millis();
        }

        // Check if rotation complete
        if (checkRotation()) {
          // Rotation done, finalize calibration
          stopWithPID();
          magnetometer.finalizeCalibration();

          // Re-initialize pose after calibration spin
          pose.initialise(0, 0, 0);
          pose_update_ts = millis();

          // Start grace period: disable puck detection until magnetometer settles
          post_calibration_grace_end = millis() + POST_CALIBRATION_GRACE_MS;

          // Move to next state: travel to first waypoint
          mag_calibration_in_progress = false;
          robot_state = STATE_TRAVEL_TO_WAYPOINT;
          travelToXY(waypoint_x[waypoint_order[0]], waypoint_y[waypoint_order[0]]);
        }
      }
      break;

    // --------------------------------------------------
    // TRAVEL_TO_WAYPOINT – navigate toward the next puck
    // --------------------------------------------------
    case STATE_TRAVEL_TO_WAYPOINT:
      if (detectPuck()) {
        // ---- PUCK FOUND: compute traverse waypoints ----
        // Start by reversing away from the puck
        setReverse(REVERSE_DISTANCE);
        traverse_phase = TRAVERSE_REVERSE;
        robot_state = STATE_TRAVERSE_PUCKED_WAYPOINT;

      } else {
        // updateTravelToXY() returns true once the waypoint is reached
        if (updateTravelToXY()) {
          stopWithPID();
          robot_state  = STATE_AT_WAYPOINT;
          maneuver_ts  = millis();  // start the pause timer
        }
      }
      break;

    // --------------------------------------------------
    // AT_WAYPOINT – pause, then check for puck
    // --------------------------------------------------
    case STATE_AT_WAYPOINT:
      if (millis() - maneuver_ts >= WAYPOINT_PAUSE_MS) {

        if (detectPuck()) {
          // ---- PUCK FOUND: compute traverse waypoints ----
          // Start by reversing away from the puck
          setReverse(REVERSE_DISTANCE);
          traverse_phase = TRAVERSE_REVERSE;
          robot_state = STATE_TRAVERSE_PUCKED_WAYPOINT;

        } else {
          // ---- NO PUCK: advance to next waypoint ----
          current_waypoint_idx++;
          if (current_waypoint_idx >= NUM_WAYPOINTS) {
            travelToXY(0, 0);
            robot_state = STATE_RETURN_HOME;
            current_waypoint_idx = 0;  // wrap and repeat the circuit
            break;
          }

          travelToXY(waypoint_x[waypoint_order[current_waypoint_idx]],
                     waypoint_y[waypoint_order[current_waypoint_idx]]);

          robot_state = STATE_TRAVEL_TO_WAYPOINT;
        }
      }
      break;

    // --------------------------------------------------
    // TRAVERSE_PUCKED_WAYPOINT – manoeuvre around puck,
    // position behind it on the puck→home line, then push
    // --------------------------------------------------
    case STATE_TRAVERSE_PUCKED_WAYPOINT:
      switch (traverse_phase) {

        // Phase 1: reverse a fixed distance from the puck
        case TRAVERSE_REVERSE:
          if (checkReverse()) {
            float px = waypoint_x[waypoint_order[current_waypoint_idx]];
            float py = waypoint_y[waypoint_order[current_waypoint_idx]];

            // --- Compute push position (rF) behind puck on the home→puck line ---
            // m1 = unit vector from origin (home) toward puck
            float m1_dist = sqrt(px * px + py * py);
            float m1_x = px / m1_dist;
            float m1_y = py / m1_dist;
            // rF sits TRAVERSE_OFFSET mm beyond the puck, along the home→puck line
            float rf_x = px + m1_x * TRAVERSE_OFFSET;
            float rf_y = py + m1_y * TRAVERSE_OFFSET;

            // --- Decide whether rF is reachable directly or needs a bypass ---
            // m2 = unit vector from robot toward puck
            float dx_rp = px - pose.x;
            float dy_rp = py - pose.y;
            float rp_dist = sqrt(dx_rp * dx_rp + dy_rp * dy_rp);
            float m2_x = dx_rp / rp_dist;
            float m2_y = dy_rp / rp_dist;

            // Compare the angle of the home→puck line with the robot→puck line.
            // If the angular difference is large (> PI), rF is roughly "behind"
            // the robot so it can drive there directly.  Otherwise it must
            // swing around the puck via a perpendicular bypass waypoint.
            float rf_angle = atan2(m1_y, m1_x);
            float rp_angle = atan2(m2_y, m2_x);
            float ang_diff = angleDiff(rp_angle, rf_angle);

            if (fabs(ang_diff) >= PI / 2.0) {
              // rF is roughly accessible — drive straight there
              traverse_wp_x[0] = rf_x;
              traverse_wp_y[0] = rf_y;
              in_far_quadrant = false;
            } else {
              // rF is on the far side — first swing perpendicular to
              // the robot→puck line to avoid pushing the puck off course.
              // Perpendicular of m2 = (-m2_y, m2_x); choose the side
              // that moves toward the push line (sign of ang_diff).
              float perp_x, perp_y;
              if (ang_diff > 0) {
                // Swing left of the robot→puck line
                perp_x = m2_y;
                perp_y = -m2_x;
              } else {
                // Swing right of the robot→puck line
                perp_x = -m2_y;
                perp_y = m2_x;
              }
              // Bypass WP: offset perpendicular from the puck position
              traverse_wp_x[0] = px + perp_x * TRAVERSE_OFFSET;
              traverse_wp_y[0] = py + perp_y * TRAVERSE_OFFSET;

              // Then proceed to push position behind puck
              traverse_wp_x[1] = rf_x;
              traverse_wp_y[1] = rf_y;
              in_far_quadrant = true;
            }
            travelToXY(traverse_wp_x[0], traverse_wp_y[0]);
            traverse_phase = TRAVERSE_WP1;
          }
          break;

        // Phase 2: travel to first bypass waypoint
        case TRAVERSE_WP1:
          if (updateTravelToXY()) {
            stopWithPID();
            if (in_far_quadrant) {
              travelToXY(traverse_wp_x[1], traverse_wp_y[1]);
              traverse_phase = TRAVERSE_WP2;
            } else {
              travelToXY(40, 10);
              robot_state = STATE_RETURN_HOME;
            }
          }
          break;

        // Phase 3: travel to second bypass waypoint
        case TRAVERSE_WP2:
          if (updateTravelToXY()) {
            stopWithPID();
            travelToXY(40, 10);
            robot_state = STATE_RETURN_HOME;
          }
          break;

      }
      break;

    // --------------------------------------------------
    // RETURN_HOME – navigate back to start using odometry
    // --------------------------------------------------
    case STATE_RETURN_HOME:
      // updateTravelToXY() returns true once (0,0) is reached
      if (updateTravelToXY()) {
        stopWithPID();
        // Timer expired while returning — stay at home
        robot_state = STATE_WAITING;
        waiting_ts = millis();  // start waiting timer (optional, for display)
      }
      break;
    
    case STATE_WAITING:
      if (millis() - waiting_ts >= WAITING_DURATION_MS) {
        current_waypoint_idx = 0;
        travelToXY(waypoint_x[waypoint_order[current_waypoint_idx]],
                   waypoint_y[waypoint_order[current_waypoint_idx]]);
        robot_state = STATE_TRAVEL_TO_WAYPOINT;
      }
      break;
    // --------------------------------------------------
    // STOPPED – run is finished, hold motors off
    // --------------------------------------------------
    case STATE_STOPPED:
      stopWithPID();
      break;

  } // end switch (robot_state)

  // If you are using an OLED or LCD display,
  // the following will update the display with
  // the time remaining in seconds.
  display.timeRemaining();
}