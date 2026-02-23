/*************************************************************
 * OLED_Test.ino  -  Standalone OLED display verification
 *
 * Upload this sketch (separate from the main robot sketch) to
 * confirm the OLED module is correctly wired, powered, and
 * responding to software before relying on it in the main code.
 *
 * Board: Pololu 3Pi+ 32U4  (pololu-a-star:avr:a-star32U4)
 *
 * Requires:
 *   - PololuOLED library installed via Arduino Library Manager
 *   - oled.h in THIS directory (already present alongside this file)
 *
 * What to expect (watch the OLED AND the Serial Monitor at 9600 baud):
 *
 *   Phase 1  INIT       "OLED" / "READY"     3 s
 *     -> Confirms display hardware turns on. If nothing appears here,
 *        the display is not receiving power or is seated incorrectly.
 *
 *   Phase 2  ROWS       "ROW-0" / "ROW-1"    3 s
 *     -> Confirms both display rows are independently addressable.
 *
 *   Phase 3  CHARS      "12345678"/ "ABCDEFGH"  3 s
 *     -> All 8 character columns lit on both rows.
 *        Partial display = SPI wiring issue (bad pin or loose connector).
 *
 *   Phase 4  NUMBERS    "Count:" / 0..9      0.5 s per digit
 *     -> Verifies numeric rendering on row 1, which is the same
 *        position used by timeRemaining() in the main sketch.
 *
 *   Phase 5  COUNTDOWN  "Sec:" / 60..0       60 s via timeRemaining()
 *     -> Exercises the actual timeRemaining() code path used by the
 *        robot.  Should update cleanly once per second with no flicker.
 *        Serial Monitor also prints the remaining time each second.
 *
 *   Phase 6  PASS       "TEST" / "PASS"      3 s
 *     -> All phases complete.  Loops back to Phase 5 indefinitely
 *        so the display stays live for inspection.
 *
 * If any phase shows garbage or nothing, check:
 *   1. Display orientation (single-row connector, not reversed)
 *   2. Motor power switch ON (display needs regulated logic power)
 *   3. PololuOLED library installed
 *************************************************************/

#include "oled.h"

// Blue OLED module, single row of 5 pins.
// Constructor pin order: (CLK, MOSI, RES, DC, CS)
OLED_c display(1, 30, 0, 17, 13);

// ── Test state machine ────────────────────────────────────────────────────
enum TestPhase {
  PHASE_INIT,       // Power-on check
  PHASE_ROWS,       // Both display rows addressed separately
  PHASE_CHARS,      // All 8 character positions on each row
  PHASE_NUMBERS,    // Numeric 0-9 (same format as countdown)
  PHASE_COUNTDOWN,  // Live 60-s countdown via display.timeRemaining()
  PHASE_PASS        // All checks complete; restart countdown
};
TestPhase phase = PHASE_INIT;

int  count_digit       = 0;     // digit counter for PHASE_NUMBERS
unsigned long serial_ts = 0;   // throttle Serial prints in PHASE_COUNTDOWN


// ── showScreen() ──────────────────────────────────────────────────────────
// Write two text rows to the OLED in a single SPI transfer (no flicker).
// Uses noAutoDisplay() so clear() and print() only update the internal
// buffer; display() then pushes the complete screen in one shot.
//
// NOTE: disableUSB()/enableUSB() are private to OLED_c, so we cannot call
// them here.  display() itself is a fast SPI burst (~1 ms at 16 MHz) so USB
// corruption is unlikely but possible when Serial is heavily active.
// For the COUNTDOWN phase, use display.timeRemaining() instead — it has
// its own USB-safe wrapper inside oled.h.

void showScreen(const char* row0, const char* row1) {
  display.noAutoDisplay();   // buffer writes; no SPI until display() called
  display.clear();           // wipe text buffer
  display.gotoXY(0, 0);
  display.print(row0);
  display.gotoXY(0, 1);
  display.print(row1);
  display.display();         // one SPI transfer for the complete screen
}

// Overload: integer on row 1.
void showScreen(const char* row0, int value) {
  display.noAutoDisplay();
  display.clear();
  display.gotoXY(0, 0);
  display.print(row0);
  display.gotoXY(0, 1);
  display.print(value);
  display.display();
}


// ── setup() ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  delay(2000);   // allow Serial Monitor to connect

  Serial.println(F("========================================="));
  Serial.println(F("  OLED_Test: starting"));
  Serial.println(F("========================================="));

  // Initialise the display hardware.
  // display.reset() calls disableUSB() before the SPI init sequence so the
  // SH1106 Display-ON command (0xAF) is not corrupted by USB interrupts.
  // Without this call the screen may stay blank even if wiring is correct.
  display.reset();

  Serial.println(F("display.reset() complete."));
  Serial.println(F("If the screen is still completely dark, check:"));
  Serial.println(F("  - Display seated correctly (single-row connector)"));
  Serial.println(F("  - Motor power switch ON"));
  Serial.println(F("  - PololuOLED library installed"));
  Serial.println();

  phase = PHASE_INIT;
}


// ── loop() ───────────────────────────────────────────────────────────────
void loop() {
  display.print("Looping...");  // keep USB active to reduce corruption risk

  // switch (phase) {

  //   // ─────────────────────────────────────────────────────────────────────
  //   // Phase 1: INIT
  //   // The very first pixels on the screen.  If nothing appears, the
  //   // hardware is not initialising (power, orientation, or library issue).
  //   // ─────────────────────────────────────────────────────────────────────
  //   case PHASE_INIT:
  //     showScreen("OLED", "READY");
  //     Serial.println(F("Phase 1 INIT: OLED should show  'OLED' / 'READY'"));
  //     delay(3000);
  //     phase = PHASE_ROWS;
  //     break;


  //   // ─────────────────────────────────────────────────────────────────────
  //   // Phase 2: ROWS
  //   // Each row shown with its own label to verify independent addressing.
  //   // If only one row lights up, there may be a row-address offset issue
  //   // specific to this SH1106 panel revision.
  //   // ─────────────────────────────────────────────────────────────────────
  //   case PHASE_ROWS:
  //     showScreen("ROW-0", "ROW-1");
  //     Serial.println(F("Phase 2 ROWS:  OLED should show  'ROW-0' / 'ROW-1'"));
  //     delay(3000);
  //     phase = PHASE_CHARS;
  //     break;


  //   // ─────────────────────────────────────────────────────────────────────
  //   // Phase 3: CHARS
  //   // All 8 character columns on both rows.  Partial content (e.g. only
  //   // 4 chars visible) indicates a column-address offset problem or a
  //   // loose SPI data pin (MOSI, pin 30).
  //   // ─────────────────────────────────────────────────────────────────────
  //   case PHASE_CHARS:
  //     showScreen("12345678", "ABCDEFGH");
  //     Serial.println(F("Phase 3 CHARS: OLED should show  '12345678' / 'ABCDEFGH'"));
  //     delay(3000);
  //     count_digit = 0;
  //     phase = PHASE_NUMBERS;
  //     break;


  //   // ─────────────────────────────────────────────────────────────────────
  //   // Phase 4: NUMBERS
  //   // Counts 0-9 on row 1 with "Count:" label on row 0.
  //   // This is the same row/format used by timeRemaining() in the main
  //   // sketch.  Verify numbers are crisp and centred as expected.
  //   // ─────────────────────────────────────────────────────────────────────
  //   case PHASE_NUMBERS:
  //     showScreen("Count:", count_digit);
  //     Serial.print(F("Phase 4 NUMBERS: Count:"));
  //     Serial.println(count_digit);
  //     delay(500);

  //     count_digit++;
  //     if (count_digit > 9) {
  //       // Advance to countdown phase.
  //       // Use 1 minute (60 s) so the full countdown is observable.
  //       display.setMaxMinutes(1);
  //       display.startStopwatch();
  //       serial_ts = millis();
  //       Serial.println(F("Phase 5 COUNTDOWN: starting 60-s countdown via timeRemaining()"));
  //       Serial.println(F("  Row 0: 'Sec:'   Row 1: seconds remaining"));
  //       phase = PHASE_COUNTDOWN;
  //     }
  //     break;


  //   // ─────────────────────────────────────────────────────────────────────
  //   // Phase 5: COUNTDOWN
  //   // Calls display.timeRemaining() every loop iteration (non-blocking).
  //   // The function internally throttles screen updates to once per second.
  //   // Serial mirrors the approximate remaining time every 5 s so you can
  //   // verify the display and Serial agree.
  //   //
  //   // Expected: display shows "Sec:" on row 0, counting down 60..1 on
  //   //           row 1, then shows "- Done -" when timer reaches zero.
  //   // ─────────────────────────────────────────────────────────────────────
  //   case PHASE_COUNTDOWN: {
  //     bool running = display.timeRemaining();

  //     // Mirror approximate time to Serial every 5 seconds
  //     if (millis() - serial_ts >= 5000) {
  //       serial_ts = millis();
  //       Serial.println(F("Phase 5 COUNTDOWN: active (check OLED for value)"));
  //     }

  //     // timeRemaining() returns false once end_ts is reached
  //     if (!running) {
  //       Serial.println(F("Phase 5 COUNTDOWN: timer expired. Moving to PASS."));
  //       phase = PHASE_PASS;
  //     }
  //     break;
  //   }


  //   // ─────────────────────────────────────────────────────────────────────
  //   // Phase 6: PASS
  //   // All phases completed successfully.  Show result then loop the
  //   // countdown indefinitely so the display stays live for inspection.
  //   // ─────────────────────────────────────────────────────────────────────
  //   case PHASE_PASS:
  //     showScreen("TEST", "PASS");
  //     Serial.println(F("========================================="));
  //     Serial.println(F("  OLED_Test: ALL PHASES PASSED"));
  //     Serial.println(F("  Restarting countdown loop."));
  //     Serial.println(F("========================================="));
  //     delay(3000);

  //     display.setMaxMinutes(1);
  //     display.startStopwatch();
  //     serial_ts = millis();
  //     phase = PHASE_COUNTDOWN;
  //     break;

  } // end switch
}
