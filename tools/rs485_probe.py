#!/usr/bin/env python3
import os
import select
import sys
import termios
import time


BAUDS = {
    4800: termios.B4800,
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def open_port(path: str, baud: int, parity: str = "N") -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = BAUDS[baud] | termios.CS8 | termios.CREAD | termios.CLOCAL
    if parity == "E":
        attrs[2] |= termios.PARENB
    elif parity == "O":
        attrs[2] |= termios.PARENB | termios.PARODD
    attrs[3] = 0
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 2
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_for(fd: int, seconds: float) -> bytes:
    end = time.time() + seconds
    data = b""
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            try:
                chunk = os.read(fd, 256)
            except BlockingIOError:
                continue
            if chunk:
                data += chunk
                end = time.time() + 0.15
    return data


def query(fd: int, addr: int, start: int = 0, count: int = 10) -> bytes:
    frame = bytes([addr, 0x03, start >> 8, start & 0xFF, count >> 8, count & 0xFF])
    crc = crc16_modbus(frame)
    frame += bytes([crc & 0xFF, crc >> 8])
    termios.tcflush(fd, termios.TCIOFLUSH)
    os.write(fd, frame)
    return read_for(fd, 0.35)


def hexdump(data: bytes) -> str:
    return data.hex(" ").upper()


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

    print(f"Port: {path}")
    for baud in (9600, 4800, 19200, 38400, 57600, 115200):
        fd = open_port(path, baud)
        try:
            passive = read_for(fd, 1.0)
            if passive:
                print(f"PASSIVE baud={baud}: {hexdump(passive)}")

            for addr in range(1, 9):
                resp = query(fd, addr)
                if resp:
                    print(f"HIT baud={baud} addr={addr}: {hexdump(resp)}")
                    return 0
        finally:
            os.close(fd)

    print("No response on common baud rates/address 1-8.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
