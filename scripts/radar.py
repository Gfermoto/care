#!/usr/bin/env python3
"""Монитор радара HLK-LD2450, протокол AA FF."""
import argparse
import sys

import serial


def to_signed16(value: int) -> int:
    if value >= 0x8000:
        return value - 0x10000
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description='Монитор LD2450 (AA FF)')
    parser.add_argument('-p', '--port', default='/dev/ttyUSB0',
                        help='Порт (default: /dev/ttyUSB0)')
    parser.add_argument('-b', '--baud', type=int, default=256000,
                        help='Скорость (default: 256000)')
    args = parser.parse_args()

    speeds = [args.baud, 230400, 250000, 256000, 115200]
    speeds = list(dict.fromkeys(speeds))

    print('LD2450 monitor, protocol AA FF')
    print('=' * 40)

    ser = None
    for speed in speeds:
        try:
            print(f'Trying {args.port} @ {speed}...', end=' ')
            ser = serial.Serial(args.port, speed, timeout=0.1)
            print('OK')
            break
        except (serial.SerialException, OSError):
            print('fail')
            continue

    if not ser:
        print('Could not open port')
        return 1

    print(f'Connected at {ser.baudrate} baud')
    print('Waiting for AA FF packets. Ctrl+C to exit.\n')

    buffer = bytearray()
    count = 0

    try:
        while True:
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))

                while len(buffer) >= 8:
                    try:
                        idx = buffer.index(b'\xAA\xFF')
                        if idx + 8 > len(buffer):
                            break

                        x = to_signed16(buffer[idx+4] | (buffer[idx+5] << 8))
                        y = to_signed16(buffer[idx+6] | (buffer[idx+7] << 8))
                        print(f'  x:{x:5d} mm  y:{y:5d} mm')

                        count += 1
                        if count % 20 == 0:
                            print(f'  -- {count} targets --')

                        buffer = buffer[idx + 8:]
                    except ValueError:
                        buffer = buffer[1:]
                        break

    except KeyboardInterrupt:
        print(f'\nStopped. Total: {count}')
    finally:
        ser.close()
        print('Port closed')

    return 0


if __name__ == '__main__':
    sys.exit(main())
