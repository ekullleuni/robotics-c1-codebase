"""
log_sensors.py
--------------
Captures serial data from the LineSensorTest Arduino sketch
and saves it to a timestamped CSV file.

Usage:
    python log_sensors.py                  # auto-detect port
    python log_sensors.py /dev/tty.usbmodem1234  # specify port

Output files are written to  data/<YYYYMMDD_HHMMSS>.csv
Comment lines (starting with #) are echoed to the terminal but also
saved in the file so you can reconstruct what happened.

Press Ctrl+C to stop at any time; the file is safely closed first.
"""

import serial
import serial.tools.list_ports
import sys
import os
import datetime

# -----------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------
BAUD_RATE   = 115200          # Must match LineSensorTest.ino
OUTPUT_DIR  = "data"          # Folder for saved CSV files (created if absent)
ENCODING    = "utf-8"         # Serial character encoding


# -----------------------------------------------------------------------
# Port detection
# -----------------------------------------------------------------------
def find_port():
    """Return the first USB serial port that looks like an Arduino."""
    ports = list(serial.tools.list_ports.comports())
    candidates = [p for p in ports if "usbmodem" in p.device.lower()
                                    or "usbserial" in p.device.lower()
                                    or "arduino"   in p.description.lower()
                                    or "tty.usb"   in p.device.lower()]
    if not candidates:
        # Fall back to listing everything so the user can choose
        print("No Arduino/USB serial port auto-detected.")
        print("Available ports:")
        for p in ports:
            print(f"  {p.device}  —  {p.description}")
        sys.exit(1)

    if len(candidates) == 1:
        return candidates[0].device

    # Multiple candidates — pick the first and warn
    print("Multiple USB serial ports found; using the first:")
    for p in candidates:
        print(f"  {p.device}  —  {p.description}")
    return candidates[0].device


# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
def main():
    # Resolve serial port
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    print(f"Connecting to {port} at {BAUD_RATE} baud …")

    # Create output directory
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Build a timestamped filename
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename  = os.path.join(OUTPUT_DIR, f"{timestamp}.csv")

    print(f"Saving to: {filename}")
    print("Press Ctrl+C to stop logging.\n")
    print("─" * 60)

    data_line_count    = 0
    comment_line_count = 0

    def open_port():
        """Open serial port, retrying until successful (handles post-upload USB reset)."""
        while True:
            try:
                s = serial.Serial(port, BAUD_RATE, timeout=1)
                print(f"Connected to {port}")
                return s
            except serial.SerialException:
                print(f"Waiting for {port} …", end="\r")
                import time; time.sleep(0.5)

    ser = open_port()

    try:
        with open(filename, "w", newline="", encoding=ENCODING) as f:

            while True:
                try:
                    raw = ser.readline()
                except serial.SerialException:
                    # Device disconnected (e.g. USB reset after flash) — reconnect
                    print("\nPort lost — reconnecting …")
                    try: ser.close()
                    except Exception: pass
                    ser = open_port()
                    continue

                if not raw:
                    continue  # Timeout — loop again

                try:
                    line = raw.decode(ENCODING).rstrip("\r\n")
                except UnicodeDecodeError:
                    continue  # Skip garbled bytes on connect

                # Save every line (data + comments) to the file
                f.write(line + "\n")
                f.flush()  # Write to disk immediately — no data lost on Ctrl+C

                # Print to terminal
                if line.startswith("#"):
                    print(f"\033[90m{line}\033[0m")   # grey
                    comment_line_count += 1
                elif line.strip():
                    print(line)
                    data_line_count += 1

    except KeyboardInterrupt:
        print("\n" + "─" * 60)
        print(f"Logging stopped.")
        print(f"  Data rows saved : {data_line_count}")
        print(f"  Comment lines   : {comment_line_count}")
        print(f"  File            : {filename}")

    finally:
        try: ser.close()
        except Exception: pass


if __name__ == "__main__":
    main()
