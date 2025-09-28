# C.A.R.E. - Development Environment Setup

## 📋 Системные зависимости

### Python зависимости
```bash
# Установка Python пакетов для ESP-IDF
pip install --break-system-packages --upgrade setuptools
pip install --break-system-packages pkg_resources
```

### PlatformIO
```bash
# Установка PlatformIO
pip install --user platformio

# Или через pipx (рекомендуется)
pipx install platformio
```

### ESP-IDF (если используется отдельно)
```bash
# Установка ESP-IDF
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
. ./export.sh
```

## 🔧 Платформы и инструменты

### ESP32 Development
- **Platform**: Espressif 32
- **Framework**: ESP-IDF (рекомендуется) или Arduino
- **Board**: ESP32 Dev Module
- **Tools**: 
  - `toolchain-xtensa-esp32`
  - `tool-esptoolpy`
  - `tool-openocd-esp32`

### Raspberry Pi Pico Development
- **Platform**: Raspberry Pi Pico
- **Framework**: Arduino
- **Board**: Pico
- **Tools**:
  - `toolchain-pico`
  - `tool-openocd-rp2040`

### ROS 2 Development
- **ROS 2**: Humble
- **Dependencies**:
  - `rclcpp`
  - `std_msgs`
  - `sensor_msgs`
  - `geometry_msgs`
  - `ament_cmake`
  - `rosidl_default_generators`

## 🚀 Быстрая настройка

### 1. Клонирование репозитория
```bash
git clone <repository-url>
cd CARE
```

### 2. Установка зависимостей
```bash
# Python зависимости
pip install --break-system-packages --upgrade setuptools

# PlatformIO (если не установлен)
pip install --user platformio
```

### 3. Компиляция ESP32
```bash
cd esp32
pio run
```

### 4. Компиляция Raspberry Pi Pico
```bash
cd rpi_pico
pio run
```

### 5. Сборка ROS 2
```bash
cd ros2
colcon build
```

## 🛠️ Решение проблем

### Ошибка: `ModuleNotFoundError: No module named 'pkg_resources'`
```bash
pip install --break-system-packages --upgrade setuptools
```

### Ошибка: `CONFIG_FREERTOS_HZ redefined`
- Удалить `-DCONFIG_FREERTOS_HZ=1000` из `build_flags` в `platformio.ini`

### Ошибка: `undefined reference to 'app_main'`
- Добавить `extern "C"` перед `void app_main(void)`

### Ошибка: `fatal: not a git repository`
```bash
git init
git add .
git commit -m "Initial commit"
```

## 📁 Структура проекта

```
CARE/
├── esp32/                 # ESP32 проект (PlatformIO)
│   ├── src/main.cpp      # Основной код ESP32
│   ├── include/          # Заголовочные файлы
│   └── platformio.ini    # Конфигурация PlatformIO
├── rpi_pico/             # Raspberry Pi Pico проект
│   ├── src/main.cpp      # Основной код Pico
│   ├── include/          # Заголовочные файлы
│   └── platformio.ini    # Конфигурация PlatformIO
├── ros2/                 # ROS 2 пакеты
│   └── care_ld2450_driver/
│       ├── src/          # ROS 2 узлы
│       ├── msg/          # Пользовательские сообщения
│       ├── srv/          # Пользовательские сервисы
│       └── launch/       # Launch файлы
└── docs/                 # Документация
```

## 🔌 Аппаратные требования

### ESP32
- **Микроконтроллер**: ESP32 DevKit
- **Радар**: Hi-Link LD2450 (24 GHz FMCW)
- **CAN**: Встроенный TWAI + CAN трансивер
- **Подключение**:
  - LD2450: UART2 (GPIO16/17)
  - CAN: TWAI (GPIO21/22)

### Raspberry Pi Pico
- **Микроконтроллер**: Raspberry Pi Pico
- **Радар**: Hi-Link LD2450 (24 GHz FMCW)
- **CAN**: Pico-CAN-B (MCP2515 + трансивер)
- **Подключение**:
  - LD2450: UART
  - CAN: SPI (MCP2515)

## 📚 Дополнительные ресурсы

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [PlatformIO Documentation](https://docs.platformio.org/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [LD2450 Radar Documentation](https://wiki.seeedstudio.com/24GHz_Human_Detection_Radar_Sensor_LD2450/)

## 🐛 Отладка

### ESP32
```bash
# Мониторинг через UART
pio device monitor

# Загрузка прошивки
pio run --target upload

# Очистка проекта
pio run --target clean
```

### ROS 2
```bash
# Запуск узла
ros2 run care_ld2450_driver ld2450_driver_node

# Просмотр топиков
ros2 topic list
ros2 topic echo /radar/targets
```

## 📝 Примечания

- Используйте ESP-IDF framework для максимальной производительности ESP32
- Для Raspberry Pi Pico требуется внешний CAN контроллер
- ROS 2 узлы работают на хост-системе, получая данные от ESP32/Pico
- Все проекты используют PlatformIO для удобства разработки
