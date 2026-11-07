#!/usr/bin/env python3
"""Send periodic CAN frames through the USB-CAN bridge."""

import argparse
import os
import select
import termios
import time


class SlcanPort:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.original_attributes = termios.tcgetattr(self.fd)
        self.receive_buffer = bytearray()

        attributes = termios.tcgetattr(self.fd)
        attributes[0] = 0
        attributes[1] = 0
        attributes[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attributes[3] = 0
        attributes[4] = termios.B115200
        attributes[5] = termios.B115200
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attributes)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.original_attributes)
        os.close(self.fd)

    def write_command(self, command):
        remaining = memoryview((command + "\r").encode("ascii"))
        deadline = time.monotonic() + 1.0

        while remaining:
            timeout = deadline - time.monotonic()
            if timeout <= 0:
                raise TimeoutError("serial port is not writable")

            _, writable, _ = select.select([], [self.fd], [], timeout)
            if not writable:
                raise TimeoutError("serial port is not writable")

            written = os.write(self.fd, remaining)
            remaining = remaining[written:]

    def read_lines(self, timeout):
        lines = []
        deadline = time.monotonic() + timeout

        while True:
            delimiter_positions = [
                position
                for delimiter in (b"\r", b"\n")
                if (position := self.receive_buffer.find(delimiter)) >= 0
            ]
            if delimiter_positions:
                delimiter_position = min(delimiter_positions)
                line = bytes(self.receive_buffer[:delimiter_position])
                delimiter = self.receive_buffer[delimiter_position]
                del self.receive_buffer[:delimiter_position + 1]
                if delimiter == ord("\r") and self.receive_buffer.startswith(b"\n"):
                    del self.receive_buffer[:1]
                lines.append(line.decode("ascii", errors="replace"))
                continue

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break

            readable, _, _ = select.select([self.fd], [], [], remaining)
            if not readable:
                break

            try:
                received = os.read(self.fd, 256)
            except BlockingIOError:
                continue
            if not received:
                break
            self.receive_buffer.extend(received)

        return lines


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="seconds between frames (default: 1)")
    parser.add_argument("--count", type=int, default=0,
                        help="number of frames; 0 runs until Ctrl+C")
    parser.add_argument("--id", type=lambda value: int(value, 0), default=None,
                        help="11-bit CAN ID; required")
    parser.add_argument("--data", default="00",
                        help="frame payload as hex bytes (default: 00)")
    parser.add_argument("--confirm-bus", action="store_true",
                        help="confirm that transmitting on the connected CAN bus is intended")
    args = parser.parse_args()

    if args.interval <= 0:
        parser.error("--interval must be greater than zero")
    if args.count < 0:
        parser.error("--count cannot be negative")
    if not args.confirm_bus:
        parser.error("CAN bus transmission requires --confirm-bus")
    if args.id is None:
        parser.error("an explicit --id is required")

    args.frame_id = args.id
    if not 0 <= args.frame_id <= 0x7FF:
        parser.error("--id must be an 11-bit standard CAN ID")

    try:
        args.payload = bytes.fromhex(args.data)
    except ValueError:
        parser.error("--data must contain hexadecimal byte pairs")
    if len(args.payload) > 8:
        parser.error("--data cannot be longer than 8 bytes")

    return args


def main():
    args = parse_arguments()
    port = SlcanPort(args.port)
    channel_open = False

    try:
        port.write_command("S5")
        speed_response = port.read_lines(0.5)
        if "" not in speed_response:
            raise RuntimeError("no acknowledgement for S5 (250 kbit/s)")
        print("CAN bitrate: 250 kbit/s")

        port.write_command("O")
        mode_response = port.read_lines(0.5)
        if "" not in mode_response:
            raise RuntimeError("no acknowledgement for O")
        channel_open = True
        print(f"CAN mode: active; ID: 0x{args.frame_id:03X}")

        sent_count = 0
        while args.count == 0 or sent_count < args.count:
            frame = (
                f"t{args.frame_id:03X}{len(args.payload):X}"
                f"{args.payload.hex().upper()}"
            )
            port.write_command(frame)
            print(f"TX {frame}", flush=True)

            for response in port.read_lines(args.interval):
                if response == "z":
                    print("  ACK: accepted")
                elif response == "\a":
                    print("  NACK: rejected")
                elif response:
                    print(f"  RX {response}")

            sent_count += 1

    except KeyboardInterrupt:
        print("\nStopping test.")
    finally:
        if channel_open:
            try:
                port.write_command("C")
                port.read_lines(0.5)
            except (OSError, TimeoutError):
                pass
        port.close()


if __name__ == "__main__":
    main()