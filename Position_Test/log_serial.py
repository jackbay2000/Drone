"""
Logs Position_Test.ino's CSV serial output straight to a timestamped .csv
file -- no manual copy-paste from the Serial Monitor needed.

Usage:
    python log_serial.py                  # auto-detect port, log until Ctrl+C
    python log_serial.py --port COM5
    python log_serial.py --port COM5 --seconds 60   # stop automatically after 60s

Requires pyserial:  pip install pyserial
"""

import argparse
import datetime
import os
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("This needs pyserial. Install it with:\n    pip install pyserial")
    sys.exit(1)

_BAUD = 115200
_OUT_DIR = "logs"

# Must match Position_Test.ino's CSV_HEADER exactly. Written to the output
# file ourselves rather than relying on catching it off the wire -- the
# sketch only prints it once, at boot, and if the board was already running
# before this script connected (e.g. it wasn't power-cycled, or a previous
# connection attempt failed after the board had already booted), that one
# print has already scrolled past and will never be seen here.
_CSV_HEADER = "t_ms,x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg"
_N_FIELDS   = _CSV_HEADER.count(',') + 1


def _is_data_row(line: str) -> bool:
    """True if `line` parses as N comma-separated numbers -- i.e. it's an
    actual telemetry row, not a boot message or the sketch's own header."""
    parts = line.split(',')
    if len(parts) != _N_FIELDS:
        return False
    try:
        int(parts[0])
        for p in parts[1:]:
            float(p)
        return True
    except ValueError:
        return False


def pick_port() -> str:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Is the board plugged in?")
        sys.exit(1)
    if len(ports) == 1:
        print(f"Using {ports[0].device} ({ports[0].description})")
        return ports[0].device
    print("Multiple serial ports found:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  ({p.description})")
    choice = input("Pick a port number: ").strip()
    return ports[int(choice)].device


def main():
    parser = argparse.ArgumentParser(description="Log Position_Test.ino telemetry to CSV")
    parser.add_argument('--port', default=None, help='Serial port (e.g. COM5); auto-detected if omitted')
    parser.add_argument('--baud', type=int, default=_BAUD)
    parser.add_argument('--seconds', type=float, default=None,
                        help='Stop automatically after this many seconds (default: run until Ctrl+C)')
    parser.add_argument('--out', default=None, help='Output CSV path (default: logs/position_test_<timestamp>.csv)')
    args = parser.parse_args()

    port = args.port or pick_port()

    os.makedirs(_OUT_DIR, exist_ok=True)
    out_path = args.out or os.path.join(
        _OUT_DIR, f"position_test_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

    print(f"Connecting to {port} @ {args.baud}...")
    try:
        ser = serial.Serial(port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"\nCouldn't open {port}: {e}\n")
        if 'Access is denied' in str(e) or 'PermissionError' in str(e):
            print("This almost always means something else already has the port open --")
            print("most commonly the Arduino IDE's Serial Monitor left open from uploading.")
            print("Close that (or close Arduino IDE entirely) and try again.")
        sys.exit(1)
    time.sleep(2.0)  # let the board reset after the port opens (typical on Arduino/Teensy)

    print(f"Logging to {out_path}")
    print("Waiting for board... (Ctrl+C to stop)" if args.seconds is None
          else f"Waiting for board... (stopping automatically after {args.seconds:.0f}s)")

    n_rows = 0
    t_start = None
    try:
        with open(out_path, 'w', newline='') as f:
            f.write(_CSV_HEADER + '\n')   # written ourselves -- see _CSV_HEADER comment above
            while True:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue

                if line.startswith('t_ms'):
                    # The sketch's own one-time header -- already covered above, just note it.
                    print(f"  (sketch header seen: {line})")
                    continue

                if not _is_data_row(line):
                    # Boot/calibration message or a '#' comment -- shown so
                    # the run doesn't look frozen, not written to the file.
                    print(f"  [info] {line}")
                    continue

                f.write(line + '\n')
                f.flush()
                n_rows += 1
                if t_start is None:
                    t_start = time.time()
                if n_rows % 10 == 0:
                    print(f"  {n_rows} rows logged...")

                if args.seconds is not None and t_start is not None and (time.time() - t_start) >= args.seconds:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    print(f"\nDone. {n_rows} rows saved to {out_path}")


if __name__ == '__main__':
    main()
