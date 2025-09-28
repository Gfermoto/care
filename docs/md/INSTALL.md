# C.A.R.E. - Установка и настройка

## 🚀 Быстрая установка

### 1. Клонирование репозитория
```bash
git clone <repository-url>
cd CARE
```

### 2. Установка Python зависимостей
```bash
# Установка зависимостей
pip install -r requirements.txt

# Или вручную
pip install --break-system-packages --upgrade setuptools
pip install --break-system-packages pkg_resources
```

### 3. Установка PlatformIO
```bash
# Через pip
pip install --user platformio

# Или через pipx (рекомендуется)
pipx install platformio
```

### 4. Проверка установки
```bash
# Проверка PlatformIO
pio --version

# Проверка Python пакетов
python -c "import pkg_resources; print('OK')"
```

## 🔧 Настройка для ESP32

### Компиляция проекта
```bash
cd esp32
pio run
```

### Загрузка на устройство
```bash
# Подключите ESP32 через USB
pio run --target upload

# Мониторинг через UART
pio device monitor
```

## 🔧 Настройка для Raspberry Pi Pico

### Компиляция проекта
```bash
cd rpi_pico
pio run
```

### Загрузка на устройство
```bash
# Подключите Pico в режиме BOOTSEL
pio run --target upload
```

## 🔧 Настройка для ROS 2

### Установка ROS 2 Humble
```bash
# Ubuntu 22.04
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep

# Инициализация rosdep
sudo rosdep init
rosdep update
```

### Сборка ROS 2 пакетов
```bash
cd ros2
colcon build
source install/setup.bash
```

### Запуск системы
```bash
# Запуск радарного узла
ros2 launch care_ld2450_driver care.launch.py
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

## 📋 Системные требования

### Минимальные требования
- **OS**: Ubuntu 20.04+ / Windows 10+ / macOS 10.15+
- **Python**: 3.8+
- **RAM**: 4GB+
- **Storage**: 2GB+

### Рекомендуемые требования
- **OS**: Ubuntu 22.04 LTS
- **Python**: 3.10+
- **RAM**: 8GB+
- **Storage**: 10GB+

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

- [DEVELOPMENT.md](DEVELOPMENT.md) - Подробное руководство по разработке
- [README.md](README.md) - Общее описание проекта
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [PlatformIO Documentation](https://docs.platformio.org/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)

## 🆘 Поддержка

Если у вас возникли проблемы:

1. Проверьте [DEVELOPMENT.md](DEVELOPMENT.md) для решения типичных проблем
2. Убедитесь, что все зависимости установлены
3. Проверьте версии Python и PlatformIO
4. Создайте issue в репозитории с описанием проблемы
