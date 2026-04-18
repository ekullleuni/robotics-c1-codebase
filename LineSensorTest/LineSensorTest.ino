/******************************************
 *  LINE SENSOR SHADE CALIBRATION SKETCH
 *
 *  Stripped firmware for characterising the
 *  5 downward IR reflectance sensors against
 *  a printed grayscale shade card.
 *
 *  Procedure:
 *    1. Place the robot flat on a shade patch
 *       (DN3 centred over the patch).
 *    2. Press BUTTON A -> 20 raw ADC readings
 *       are emitted as CSV lines over serial.
 *    3. Move to the next (lighter) patch and
 *       repeat.  shade_id increments on each
 *       completed capture.
 *
 *  CSV format (data rows):
 *    shade_id, sample, s0, s1, s2, s3, s4
 *
 *  Serial: 115200 baud
 ******************************************/

#include "LineSensors.h"

// ============================================
// PIN DEFINITIONS
// ============================================
#define BUZZER_PIN    6
#define BUTTON_A_PIN  14   // Active-low with internal pull-up

// ============================================
// TEST CONFIGURATION
// ============================================
#define SAMPLES_PER_SHADE  20    // Readings per shade press
#define SAMPLE_DELAY_MS     5    // ADC settling time between samples (ms)

// ============================================
// STATE MACHINE
// ============================================
enum TestPhase { TEST_IDLE, TEST_READING };
TestPhase test_phase = TEST_IDLE;

// ============================================
// HARDWARE INSTANCE
// ============================================
LineSensors_c line_sensors;

// ============================================
// CAPTURE STATE
// ============================================
int shade_id     = 0;   // increments after each completed capture
int sample_count = 0;   // 0..SAMPLES_PER_SHADE-1 during READING

// ============================================
// BUTTON DEBOUNCE
// ============================================
unsigned long btn_a_last = 0;
bool btn_a_prev = true;        // HIGH = not pressed (pull-up)
#define DEBOUNCE_MS 200

// Edge-detect helper: true on the HIGH->LOW transition
bool buttonPressed(int pin, bool &prev_state, unsigned long &last_time) {
  bool current = digitalRead(pin);
  if (!current && prev_state && (millis() - last_time >= DEBOUNCE_MS)) {
    last_time  = millis();
    prev_state = current;
    return true;
  }
  prev_state = current;
  return false;
}

bool buttonA() { return buttonPressed(BUTTON_A_PIN, btn_a_prev, btn_a_last); }

// ============================================
// CSV OUTPUT HELPER
// ============================================
// Print raw ADC readings for all 5 sensors as CSV fields
void printSensorCSV() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(line_sensors.readings[i], 0);
    if (i < NUM_SENSORS - 1) Serial.print(F(","));
  }
}

// ============================================
// SETUP
// ============================================
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  // Line sensors in ADC mode (IR LED on, pins set to INPUT_PULLUP)
  line_sensors.initialiseForADC();

  Serial.begin(115200);
  delay(1500);  // Give USB CDC time to enumerate

  // Welcome banner (comments prefixed with '#' for the Python logger)
  Serial.println(F("# LINE SENSOR SHADE CALIBRATION"));
  Serial.println(F("# =============================="));
  Serial.println(F("# shade_id 0 = BLACK (darkest patch)"));
  Serial.println(F("# Increasing shade_id = progressively lighter"));
  Serial.println(F("# Final shade = WHITE (lightest patch)"));
  Serial.println(F("# Place robot on shade, press BUTTON A to capture 20 readings."));
  Serial.println(F("# CSV format: shade_id, sample, s0, s1, s2, s3, s4"));
  Serial.println(F("#"));
  Serial.println(F("# Ready. Place robot on shade 0 (BLACK) and press Button A."));

  tone(BUZZER_PIN, 440, 100);
}

// ============================================
// LOOP
// ============================================
void loop() {
  bool a_pressed = buttonA();

  switch (test_phase) {

    // --------------------------------------------
    // IDLE: waiting for Button A to trigger capture
    // --------------------------------------------
    case TEST_IDLE:
      if (a_pressed) {
        sample_count = 0;
        test_phase   = TEST_READING;
        tone(BUZZER_PIN, 660, 50);
      }
      break;

    // --------------------------------------------
    // READING: take SAMPLES_PER_SHADE ADC samples
    // --------------------------------------------
    case TEST_READING: {
      line_sensors.readSensorsADC();

      // CSV: shade_id, sample, s0, s1, s2, s3, s4
      Serial.print(shade_id);
      Serial.print(F(","));
      Serial.print(sample_count);
      Serial.print(F(","));
      printSensorCSV();
      Serial.println();

      sample_count++;

      if (sample_count < SAMPLES_PER_SHADE) {
        // Short blocking delay is acceptable — this is a characterisation
        // sketch, not the main robot firmware.
        delay(SAMPLE_DELAY_MS);
      } else {
        // Capture complete — advance shade and return to IDLE
        Serial.print(F("# Shade "));
        Serial.print(shade_id);
        Serial.println(F(" complete. Place robot on next shade and press Button A."));
        shade_id++;
        tone(BUZZER_PIN, 880, 100);
        test_phase = TEST_IDLE;
      }
      break;
    }
  }
}
