#!/usr/bin/env python3
"""Display encoder telemetry received from ESP32 number 1 over USB serial."""

import argparse
import glob
import os
import select
import sys
import termios
import time


def find_port(requested: str | None) -> str:
    if requested:
        return requested
    candidates = sorted(glob.glob("/dev/serial/by-id/*"))
    if not candidates:
        candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not candidates:
        raise RuntimeError("khong tim thay ESP32 (/dev/ttyUSB* hoac /dev/ttyACM*)")
    if len(candidates) > 1:
        joined = "\n  ".join(candidates)
        raise RuntimeError(f"co nhieu cong serial; hay chon bang --port:\n  {joined}")
    return candidates[0]


def open_serial(path: str, baud: int) -> int:
    baud_values = {115200: termios.B115200}
    if baud not in baud_values:
        raise RuntimeError(f"baud chua duoc ho tro: {baud}")
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = baud_values[baud]
    attrs[5] = baud_values[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def parse_frame(line: str):
    fields = line.strip().split(",")
    if len(fields) != 10 or fields[0] != "ENC":
        return None
    return int(fields[1]), [int(v) for v in fields[2:6]], [float(v) for v in fields[6:10]]


def main() -> int:
    parser = argparse.ArgumentParser(description="Hien thi encoder 4 banh tu ESP32 so 1")
    parser.add_argument("--port", help="vi du /dev/serial/by-id/usb-...")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--raw", action="store_true", help="in CSV, khong ve lai bang")
    args = parser.parse_args()

    port = find_port(args.port)
    fd = open_serial(port, args.baud)
    print(f"Dang doc {port} @ {args.baud} baud (Ctrl-C de thoat)", flush=True)
    buffer = b""
    last_frame = time.monotonic()
    try:
        while True:
            readable, _, _ = select.select([fd], [], [], 1.0)
            if not readable:
                if time.monotonic() - last_frame > 2.0:
                    print("Cho du lieu... kiem tra cap USB va firmware", file=sys.stderr)
                continue
            chunk = os.read(fd, 4096)
            if not chunk:
                continue
            buffer += chunk
            while b"\n" in buffer:
                raw_line, buffer = buffer.split(b"\n", 1)
                line = raw_line.decode("ascii", errors="replace").strip()
                frame = parse_frame(line)
                if frame is None:
                    continue
                last_frame = time.monotonic()
                millis, counts, rpms = frame
                if args.raw:
                    print(line, flush=True)
                    continue
                rows = [
                    f"ESP32 time: {millis / 1000:10.1f} s",
                    "+-------+----------------+------------+",
                    "| Banh  | Count          | RPM        |",
                    "+-------+----------------+------------+",
                ]
                for name, count, rpm in zip(("FR", "FL", "BL", "BR"), counts, rpms):
                    rows.append(f"| {name:<5} | {count:>14d} | {rpm:>10.3f} |")
                rows.append("+-------+----------------+------------+")
                sys.stdout.write("\x1b[2J\x1b[H" + "\n".join(rows) + "\n")
                sys.stdout.flush()
    except KeyboardInterrupt:
        return 0
    finally:
        os.close(fd)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"Loi: {exc}", file=sys.stderr)
        raise SystemExit(1)
