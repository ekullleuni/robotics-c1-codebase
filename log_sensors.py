"""
log_sensors.py
--------------
Captures serial data from the LineSensorTest Arduino sketch
and saves it to a CSV file whose name encodes the number of
shade captures.

Usage:
    python log_sensors.py                           # auto-detect port
    python log_sensors.py --shades 25               # pre-declare expected count
    python log_sensors.py /dev/tty.usbmodem1234     # specify port
    python log_sensors.py /dev/tty.usbmodem1234 --shades 25

Output files are written to  data/<N>shades_<YYYYMMDD_HHMMSS>.csv
where <N> is the number of unique shade_id values captured.
Comment lines (starting with #) are echoed to the terminal and
also saved in the file so you can reconstruct what happened.

Press Ctrl+C to stop at any time; the temp file is closed and
renamed so no data is lost.
"""

import serial
import serial.tools.list_ports
import sys
import os
import re
import argparse
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
        print("No Arduino/USB serial port auto-detected.")
        print("Available ports:")
        for p in ports:
            print(f"  {p.device}  —  {p.description}")
        sys.exit(1)

    if len(candidates) == 1:
        return candidates[0].device

    print("Multiple USB serial ports found; using the first:")
    for p in candidates:
        print(f"  {p.device}  —  {p.description}")
    return candidates[0].device


# -----------------------------------------------------------------------
# Argument parsing
# -----------------------------------------------------------------------
def parse_args():
    """Accept an optional positional port and an optional --shades count."""
    parser = argparse.ArgumentParser(
        description="Log line-sensor calibration CSV from an Arduino sketch.",
        add_help=True,
    )
    parser.add_argument("port", nargs="?", default=None,
                        help="Serial port (auto-detected if omitted)")
    parser.add_argument("--shades", type=int, default=None,
                        help="Expected number of shade captures (optional)")
    return parser.parse_args()


# -----------------------------------------------------------------------
# Shade counting
# -----------------------------------------------------------------------
def count_shades(path):
    """Count unique shade_id values in the CSV (skips comments/blank lines)."""
    ids = set()
    try:
        with open(path, "r", encoding=ENCODING) as f:
            for line in f:
                if line.startswith("#") or not line.strip():
                    continue
                parts = line.split(",")
                try:
                    ids.add(int(parts[0]))
                except (ValueError, IndexError):
                    continue
    except FileNotFoundError:
        return 0
    return len(ids)


# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
def main():
    args = parse_args()
    port = args.port if args.port else find_port()
    print(f"Connecting to {port} at {BAUD_RATE} baud …")
    if args.shades is not None:
        print(f"Expected shade count: {args.shades}")

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # The final filename depends on how many shades we actually captured,
    # which we don't know until Ctrl+C. Write to a temp name first and
    # rename on exit.
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    tmp_path  = os.path.join(OUTPUT_DIR, f"_tmp_{timestamp}.csv")

    print(f"Saving to (temp): {tmp_path}")
    print("Press Ctrl+C to stop logging.\n")
    print("─" * 60)

    data_line_count    = 0
    comment_line_count = 0
    shade_progress_re  = re.compile(r"#\s*Shade\s+(\d+)\s+complete", re.IGNORECASE)

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
        with open(tmp_path, "w", newline="", encoding=ENCODING) as f:

            while True:
                try:
                    raw = ser.readline()
                except serial.SerialException:
                    print("\nPort lost — reconnecting …")
                    try: ser.close()
                    except Exception: pass
                    ser = open_port()
                    continue

                if not raw:
                    continue

                try:
                    line = raw.decode(ENCODING).rstrip("\r\n")
                except UnicodeDecodeError:
                    continue

                # Persist every line immediately so Ctrl+C never loses data
                f.write(line + "\n")
                f.flush()

                # Highlight shade-complete comments so the operator can see progress
                m = shade_progress_re.match(line)
                if m:
                    done = int(m.group(1)) + 1   # shade_id is zero-indexed
                    if args.shades is not None:
                        banner = f">>> PROGRESS: {done}/{args.shades} shades captured"
                    else:
                        banner = f">>> PROGRESS: {done} shades captured"
                    print(f"\033[93m{banner}\033[0m")   # yellow

                if line.startswith("#"):
                    print(f"\033[90m{line}\033[0m")     # grey
                    comment_line_count += 1
                elif line.strip():
                    print(line)
                    data_line_count += 1

    except KeyboardInterrupt:
        print("\n" + "─" * 60)
        print("Logging stopped.")

    finally:
        try: ser.close()
        except Exception: pass

    # Finalise the filename using the actual shade count (outside finally so
    # we don't accidentally swallow exceptions from the logging loop)
    n_shades = count_shades(tmp_path)
    print(f"  Data rows saved : {data_line_count}")
    print(f"  Comment lines   : {comment_line_count}")

    if n_shades == 0:
        print(f"  No shade data captured — leaving temp file: {tmp_path}")
        return

    final_path = os.path.join(OUTPUT_DIR, f"{n_shades}shades_{timestamp}.csv")
    try:
        os.rename(tmp_path, final_path)
    except OSError as e:
        print(f"  Rename failed ({e}); data remains at {tmp_path}")
        return

    print(f"  Shades captured : {n_shades}")
    if args.shades is not None and n_shades != args.shades:
        print(f"  NOTE: expected {args.shades} shades, got {n_shades}")
    print(f"  Saved as        : {final_path}")


if __name__ == "__main__":
    main()
