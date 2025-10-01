#!/usr/bin/env python3
"""
C.A.R.E. COM Port Debug Tool
Инструмент для отладки и настройки радара и контроллера через COM порт
"""

import serial
import serial.tools.list_ports
import time
import json
import argparse
import threading
from datetime import datetime

class COMDebugger:
    def __init__(self, port=None, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        
    def list_ports(self):
        """Список доступных COM портов"""
        ports = serial.tools.list_ports.comports()
        print("🔍 Доступные COM порты:")
        for i, port in enumerate(ports):
            print(f"  {i+1}. {port.device} - {port.description}")
        return ports
    
    def connect(self, port=None):
        """Подключение к COM порту"""
        if port:
            self.port = port
            
        if not self.port:
            print("❌ Порт не указан!")
            return False
            
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            print(f"✅ Подключен к {self.port} ({self.baudrate} бод)")
            return True
            
        except Exception as e:
            print(f"❌ Ошибка подключения: {e}")
            return False
    
    def disconnect(self):
        """Отключение от COM порта"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("🔌 Отключен от COM порта")
    
    def send_command(self, command):
        """Отправка команды"""
        if not self.serial_conn or not self.serial_conn.is_open:
            print("❌ COM порт не подключен!")
            return False
            
        try:
            # Добавляем \r\n если нет
            if not command.endswith('\r\n'):
                command += '\r\n'
                
            self.serial_conn.write(command.encode())
            print(f"📤 Отправлено: {command.strip()}")
            return True
            
        except Exception as e:
            print(f"❌ Ошибка отправки: {e}")
            return False
    
    def read_response(self, timeout=2):
        """Чтение ответа"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
            
        try:
            start_time = time.time()
            response = ""
            
            while time.time() - start_time < timeout:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    response += data.decode('utf-8', errors='ignore')
                    
                    if '\n' in response:
                        break
                        
                time.sleep(0.01)
            
            if response:
                print(f"📥 Получено: {response.strip()}")
                return response.strip()
            else:
                print("⏰ Таймаут ожидания ответа")
                return None
                
        except Exception as e:
            print(f"❌ Ошибка чтения: {e}")
            return None
    
    def monitor_continuous(self):
        """Непрерывный мониторинг"""
        if not self.serial_conn or not self.serial_conn.is_open:
            print("❌ COM порт не подключен!")
            return
            
        print("📡 Непрерывный мониторинг (Ctrl+C для выхода)...")
        self.running = True
        
        try:
            while self.running:
                if self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    print(f"[{timestamp}] {data.decode('utf-8', errors='ignore').strip()}")
                    
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n🛑 Мониторинг остановлен")
            self.running = False
    
    def configure_radar(self):
        """Настройка радара LD2450"""
        print("📡 Настройка радара LD2450...")
        
        # Команды настройки LD2450
        commands = [
            "AT+VERSION",           # Версия
            "AT+MAXTARGETS=3",      # Максимум целей
            "AT+RANGE=8000",        # Дальность 8м
            "AT+SENSITIVITY=medium", # Чувствительность
            "AT+UPDATE=20",         # Частота 20 Гц
            "AT+SAVE",              # Сохранение
            "AT+RESET"              # Перезагрузка
        ]
        
        for cmd in commands:
            self.send_command(cmd)
            time.sleep(0.5)
            self.read_response()
            time.sleep(0.5)
    
    def configure_controller(self):
        """Настройка контроллера"""
        print("🎛️ Настройка контроллера...")
        
        # Команды настройки контроллера
        commands = [
            "CONFIG:CAN:500000",    # CAN 500 кбит/с
            "CONFIG:SAFETY:1000:30", # Безопасность 1м, ±15°
            "CONFIG:RADAR:UART2:256000", # Радар UART2, 256к
            "CONFIG:SAVE",          # Сохранение
            "CONFIG:RESET"          # Перезагрузка
        ]
        
        for cmd in commands:
            self.send_command(cmd)
            time.sleep(0.5)
            self.read_response()
            time.sleep(0.5)
    
    def test_radar(self):
        """Тестирование радара"""
        print("🧪 Тестирование радара...")
        
        # Команды тестирования
        commands = [
            "AT+STATUS",            # Статус
            "AT+TARGETS",           # Текущие цели
            "AT+TEST",              # Тест
            "AT+INFO"               # Информация
        ]
        
        for cmd in commands:
            self.send_command(cmd)
            time.sleep(1)
            self.read_response()
            time.sleep(1)
    
    def test_controller(self):
        """Тестирование контроллера"""
        print("🧪 Тестирование контроллера...")
        
        # Команды тестирования
        commands = [
            "STATUS",               # Статус системы
            "CAN:TEST",             # Тест CAN
            "RADAR:STATUS",          # Статус радара
            "SAFETY:STATUS"         # Статус безопасности
        ]
        
        for cmd in commands:
            self.send_command(cmd)
            time.sleep(1)
            self.read_response()
            time.sleep(1)

def main():
    parser = argparse.ArgumentParser(description='C.A.R.E. COM Port Debug Tool')
    parser.add_argument('--port', '-p', help='COM порт (например: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', '-b', type=int, default=115200, help='Скорость (по умолчанию: 115200)')
    parser.add_argument('--list', '-l', action='store_true', help='Список доступных портов')
    parser.add_argument('--monitor', '-m', action='store_true', help='Непрерывный мониторинг')
    parser.add_argument('--configure-radar', action='store_true', help='Настройка радара')
    parser.add_argument('--configure-controller', action='store_true', help='Настройка контроллера')
    parser.add_argument('--test-radar', action='store_true', help='Тестирование радара')
    parser.add_argument('--test-controller', action='store_true', help='Тестирование контроллера')
    parser.add_argument('--command', '-c', help='Отправить команду')
    
    args = parser.parse_args()
    
    debugger = COMDebugger(args.port, args.baudrate)
    
    try:
        if args.list:
            debugger.list_ports()
            return
        
        if not args.port:
            print("❌ Укажите COM порт с --port")
            debugger.list_ports()
            return
        
        if not debugger.connect():
            return
        
        if args.monitor:
            debugger.monitor_continuous()
        elif args.configure_radar:
            debugger.configure_radar()
        elif args.configure_controller:
            debugger.configure_controller()
        elif args.test_radar:
            debugger.test_radar()
        elif args.test_controller:
            debugger.test_controller()
        elif args.command:
            debugger.send_command(args.command)
            debugger.read_response()
        else:
            # Интерактивный режим
            print("🔧 C.A.R.E. COM Debug Tool - Интерактивный режим")
            print("Команды: monitor, configure-radar, configure-controller, test-radar, test-controller, exit")
            
            while True:
                try:
                    cmd = input("CARE> ").strip()
                    
                    if cmd.lower() == 'exit':
                        break
                    elif cmd.lower() == 'monitor':
                        debugger.monitor_continuous()
                    elif cmd.lower() == 'configure-radar':
                        debugger.configure_radar()
                    elif cmd.lower() == 'configure-controller':
                        debugger.configure_controller()
                    elif cmd.lower() == 'test-radar':
                        debugger.test_radar()
                    elif cmd.lower() == 'test-controller':
                        debugger.test_controller()
                    elif cmd:
                        debugger.send_command(cmd)
                        debugger.read_response()
                        
                except KeyboardInterrupt:
                    print("\n🛑 Выход...")
                    break
    
    finally:
        debugger.disconnect()

if __name__ == '__main__':
    main()
