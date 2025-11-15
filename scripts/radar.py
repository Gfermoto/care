#!/usr/bin/env python3
"""Монитор радара HLK-LD2450 - ПРАВИЛЬНЫЙ протокол"""
import serial
import sys

print("📡 Радар HLK-LD2450")
print("Скорость: 256000 бод")
print("Протокол: AA FF")
print("=" * 50)

# Пробуем разные скорости близкие к 256000
speeds = [230400, 250000, 256000, 115200]
ser = None

for speed in speeds:
    try:
        print(f"Пробуем {speed} бод...", end=" ")
        ser = serial.Serial('/dev/ttyUSB0', speed, timeout=0.1)
        print("✅")
        break
    except:
        print("❌")
        continue

if not ser:
    print("❌ Не удалось открыть порт")
    sys.exit(1)

print(f"✅ Подключено на {ser.baudrate} бод")
print("Ищем данные радара (AA FF)...")
print("Нажмите Ctrl+C для выхода\n")

buffer = bytearray()
count = 0

try:
    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            buffer.extend(data)
            
            # Ищем заголовок AA FF
            while len(buffer) >= 8:
                try:
                    # Ищем начало пакета AA FF
                    idx = buffer.index(b'\xAA\xFF')
                    
                    # Проверяем что есть достаточно данных
                    if idx + 8 <= len(buffer):
                        # Извлекаем X и Y
                        x_low = buffer[idx + 4]
                        x_high = buffer[idx + 5]
                        y_low = buffer[idx + 6]
                        y_high = buffer[idx + 7]
                        
                        x_distance = x_low | (x_high << 8)
                        y_distance = y_low | (y_high << 8)
                        
                        # Обработка знака согласно протоколу
                        if x_high & 0x80:  # Положительный X
                            x_distance -= 0x8000
                            y_distance -= 0x8000
                            print(f"🎯 x:{x_distance:5d} мм, y:{y_distance:5d} мм")
                        else:  # Отрицательный X
                            y_distance -= 0x8000
                            print(f"🎯 x:{x_distance:5d} мм, y:{y_distance:5d} мм")
                        
                        count += 1
                        if count % 20 == 0:
                            print(f"--- Обработано {count} целей ---")
                        
                        # Удаляем обработанные данные
                        buffer = buffer[idx + 8:]
                    else:
                        break
                        
                except ValueError:
                    # AA FF не найден, удаляем первый байт
                    buffer = buffer[1:]
                    break

except KeyboardInterrupt:
    print(f"\n⏹️ Остановлено. Обработано целей: {count}")
finally:
    ser.close()
    print("🔌 Порт закрыт")
