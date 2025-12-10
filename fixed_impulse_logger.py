#!/usr/bin/env python3
"""
Experiment 2: Pendulum oscillates at equilibrium with constant impulse applied each cycle.
The data generated here is intended to log the pendulum's oscillation amplitude over fixed cycle lengths.

Usage:
  python fixed_impulse_logger.py -p /dev/tty.usbserial-42310 -b 115200 -is 1.00 -cl 10
"""

import argparse
import re
import signal
import sys
import time
from pathlib import Path

import numpy as np
import serial

# Regex to match e.g. "Peak value: -1180, Timestamp: 14 ms"
RE_PEAK = re.compile(r"Peak value:\s*([-+]?\d+)\s*,\s*Timestamp:\s*([-+]?\d+)\s*ms", re.IGNORECASE)
running = True

def signal_handler(sig, frame):
    global running
    running = False

def save_segment(out_path: Path, segment_number: int, values, timestamps_ms, seen_times, impulse_strength):
    """Save a single segment to disk."""
    values_arr = np.array(values, dtype=np.int32)
    timestamps_arr = np.array(timestamps_ms, dtype=np.int64)
    seen_arr = np.array(seen_times, dtype=np.float64)
    timestamp = int(time.time())  # Add a unique UNIX timestamp to the file name

    # Ensure the directory exists (no need to append impulse_strength again)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    segment_path = out_path.parent / f"{out_path.stem}_{timestamp}_segment_{segment_number}{out_path.suffix}"
    if segment_path.suffix.lower() in (".npz",):
        np.savez_compressed(segment_path, peaks=values_arr, timestamps_ms=timestamps_arr, received_unix=seen_arr)
        print(f"Saved {len(values_arr)} entries to {segment_path} (npz)")
    elif segment_path.suffix.lower() in (".npy",):
        stacked = np.vstack([values_arr, timestamps_arr]).T
        np.save(segment_path, stacked)
        print(f"Saved {len(values_arr)} entries to {segment_path} (npy)")
    else:
        import csv
        with open(segment_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["peak_value", "timestamp_ms", "received_unix"])
            for v, t, s in zip(values_arr, timestamps_arr, seen_arr):
                writer.writerow([int(v), int(t), float(s)])
        print(f"Saved {len(values_arr)} entries to {segment_path} (csv)")

def main():
    parser = argparse.ArgumentParser(description="Monitor serial and extract Peak value logs.")
    parser.add_argument("--port", "-p", required=True, help="Serial device (e.g. /dev/tty.usbserial-42310)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--timeout", "-t", type=float, default=99.0, help="Serial read timeout (seconds).")
    parser.add_argument("--cycle_length", "-cl", type=int, default=50, help="Number of cycles per segment.")
    parser.add_argument("--impulse_strength", "-is", type=float, required=True, help="Impulse strength value (e.g., 1.00 for 1.00V).")

    args = parser.parse_args()

    port = args.port
    baud = args.baud
    # Automatically set the output path based on impulse strength
    impulse_strength = args.impulse_strength
    out_path = Path(f"logs/{impulse_strength:.2f}/peaks.npz")
    cycle_length = args.cycle_length

    # Ensure logs directory exists
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Prepare storage for segments
    values = []
    timestamps_ms = []
    seen_times = []
    segment_count = 0

    # Setup signal handlers for graceful exit
    signal.signal(signal.SIGINT, signal_handler)
    try:
        signal.signal(signal.SIGTERM, signal_handler)
    except Exception:
        pass

    print(f"Opening serial port {port} @ {baud} baud ...")
    try:
        ser = serial.Serial(port, baud, timeout=args.timeout)
    except serial.SerialException as e:
        print(f"Failed to open serial port {port}: {e}", file=sys.stderr)
        sys.exit(2)

    print("Listening for lines. Press Ctrl-C to stop and save.")
    try:
        while running:
            try:
                raw = ser.readline()
            except serial.SerialException as e:
                print(f"Serial read error: {e}", file=sys.stderr)
                break
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                line = raw.decode("latin1", errors="replace").strip()

            # Check for peak value lines
            #print(f"Received raw line: {line}")
            peak_match = RE_PEAK.search(line)
            if peak_match:
                #print(f"Received line: {line}")
                peak_val = int(peak_match.group(1))
                ts_ms = int(peak_match.group(2))
                values.append(peak_val)
                timestamps_ms.append(ts_ms)
                seen_times.append(time.time())

                # Check if cycle length is exceeded
                if len(values) >= cycle_length:
                    segment_count += 1
                    print(f"Cycle length of {cycle_length} reached. Saving segment {segment_count}.")
                    save_segment(out_path, segment_count, values, timestamps_ms, seen_times, impulse_strength)
                    values = []
                    timestamps_ms = []
                    seen_times = []
    finally:
        # ensure serial closed
        try:
            ser.close()
        except Exception:
            pass

    # Save the last segment if it has data
    if values:
        segment_count += 1
        save_segment(out_path, segment_count, values, timestamps_ms, seen_times, impulse_strength)

if __name__ == "__main__":
    main()