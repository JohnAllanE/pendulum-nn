#!/usr/bin/env python3
"""
Monitor serial port logs and extract "Peak value" and "Timestamp" into arrays/files.

Usage:
  python logger.py --port /dev/tty.usbserial-42310 --baud 115200 --out peaks.npz
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
RE_BUILD_UP = re.compile(r"build_up_mode stopped", re.IGNORECASE)
running = True

def signal_handler(sig, frame):
    global running
    running = False

def save_segment(out_path: Path, segment_number: int, values, timestamps_ms, seen_times):
    """Save a single segment to disk."""
    values_arr = np.array(values, dtype=np.int32)
    timestamps_arr = np.array(timestamps_ms, dtype=np.int64)
    seen_arr = np.array(seen_times, dtype=np.float64)
    timestamp = int(time.time())  # Add a unique UNIX timestamp to the file name
    segment_path = out_path.with_name(f"{out_path.stem}_{timestamp}_segment_{segment_number}{out_path.suffix}")
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
    parser.add_argument("--out", "-o", default="logs/peaks.npz", help="Output file (.npz, .npy or .csv). Default: logs/peaks.npz")
    parser.add_argument("--timeout", "-t", type=float, default=1.0, help="Serial read timeout (seconds).")
    parser.add_argument("--batch_size", "-bs", type=int, default=1000, help="Number of peaks to collect before saving to disk.")
    parser.add_argument("--segment_batch_size", "-sbs", type=int, default=1, help="Number of segments to collect before saving to disk.")
    args = parser.parse_args()
    port = args.port
    baud = args.baud
    out_path = Path(args.out)
    batch_size = args.batch_size
    segment_batch_size = args.segment_batch_size

    # Ensure logs directory exists
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Prepare storage for segments
    segments = []
    values = []
    timestamps_ms = []
    seen_times = []

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

            # Check for build_up mode lines
            build_up_match = RE_BUILD_UP.search(line)
            if build_up_match:
                if values:
                    # Save the current segment and start a new one
                    segments.append((values, timestamps_ms, seen_times))
                    values = []
                    timestamps_ms = []
                    seen_times = []

                    # Check if segment batch size is exceeded
                    if len(segments) >= segment_batch_size:
                        print(f"Segment batch size of {segment_batch_size} exceeded. Saving to disk.")
                        for i, segment in enumerate(segments, start=1):
                            save_segment(out_path, i, *segment)  # Save each segment in the batch
                        segments = []  # Clear the segments list after saving

                print(f"Segment boundary detected: {build_up_match.group(0)}")
                continue

            # Check for peak value lines
            peak_match = RE_PEAK.search(line)
            if peak_match:
                peak_val = int(peak_match.group(1))
                ts_ms = int(peak_match.group(2))
                values.append(peak_val)
                timestamps_ms.append(ts_ms)
                seen_times.append(time.time())

                # Check if batch size is exceeded
                if len(values) >= batch_size:
                    print(f"Batch size of {batch_size} exceeded. Saving to disk.")
                    segments.append((values, timestamps_ms, seen_times))
                    save_segment(out_path, len(segments), values, timestamps_ms, seen_times)
                    segments = []
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
        segments.append((values, timestamps_ms, seen_times))

    # Save results
    if segments:
        save_segment(out_path, len(segments), values, timestamps_ms, seen_times)

if __name__ == "__main__":
    main()