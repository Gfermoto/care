#!/usr/bin/env python3
"""Монитор радара HLK-LD2450 — протокол AA FF, signed 16-bit координаты."""
import argparse
import sys

import serial


def to_signed16(value: int) -> int:
    """Преобразование unsigned 16-bit в signed 16-bit."""
    if value >= 0x8000:
        return value - 0x10000
    return value


def main() -> int:
    """Запуск монитора радара."""
    parser = argparse.ArgumentParser(
        description='Монитор радара HLK-LD2450 (протокол AA FF)',
    )
    parser.add_argument(
        '-p', '--port',
        default='/dev/ttyUSB0',
        help='Последовательный порт (по умолчанию: /dev/ttyUSB0)',
    )
    parser.add_argument(
        '-b', '--baud',
        type=int,
        default=256000,
        help='Скорость UART (по умолчанию: 256000)',
    )
    args = parser.parse_args()

    speeds = [args.baud, 230400, 250000, 256000, 115200]
    speeds = list(dict.fromkeys(speeds))  # убираем дубликаты

    print('📡 Радар HLK-LD2450')
    print('Протокол: AA FF')
    print("=" * 50)

    ser = None
    for speed in speeds:
        try:
            msg = f'Пробуем {args.port} @ {speed} бод...'
            print(msg, end=' ')
            ser = serial.Serial(args.port, speed, timeout=0.1)
            print('✅')
            break
        except (serial.SerialException, OSError):
            print('❌')
            continue

    if not ser:
        print('❌ Не удалось открыть порт')
        return 1

    print(f'✅ Подключено на {ser.baudrate} бод')
    print('Ищем данные радара (AA FF)...')
    print('Нажмите Ctrl+C для выхода\n')

    buffer = bytearray()
    count = 0

    try:
        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                buffer.extend(data)

                while len(buffer) >= 8:
                    try:
                        idx = buffer.index(b'\xAA\xFF')

                        if idx + 8 <= len(buffer):
                            x_low = buffer[idx + 4]
                            x_high = buffer[idx + 5]
                            y_low = buffer[idx + 6]
                            y_high = buffer[idx + 7]

                            x_distance = to_signed16(x_low | (x_high << 8))
                            y_distance = to_signed16(y_low | (y_high << 8))

                            coord = f'x:{x_distance:5d} мм, y:{y_distance:5d} мм'
                            print(f'🎯 {coord}')

                            count += 1
                            if count % 20 == 0:
                                print(f'--- Обработано {count} целей ---')

                            buffer = buffer[idx + 8:]
                        else:
                            break

                    except ValueError:
                        buffer = buffer[1:]
                        break

    except KeyboardInterrupt:
        print(f'\n⏹️ Остановлено. Обработано целей: {count}')
    finally:
        ser.close()
        print('🔌 Порт закрыт')

    return 0


if __name__ == '__main__':
    sys.exit(main())
