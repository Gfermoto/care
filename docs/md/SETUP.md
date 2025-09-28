# 🛠️ C.A.R.E. - Полная инструкция по настройке и использованию

## 📋 Предварительные требования

### Системные требования:
- **Ubuntu 20.04+** или **Ubuntu 22.04+** (рекомендуется)
- **Python 3.8+**
- **Node.js 18+**
- **PlatformIO** (для микроконтроллеров)
- **ROS 2 Humble** (для робототехники)

### Установка базовых зависимостей:

```bash
# Обновление системы
sudo apt update && sudo apt upgrade -y

# Установка Python и pip
sudo apt install python3 python3-pip python3-venv -y

# Установка Node.js 18+
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install nodejs -y

# Установка PlatformIO
pip3 install platformio

# Установка ROS 2 Humble
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop -y

# Установка CAN утилит
sudo apt install can-utils -y

# Установка дополнительных инструментов
sudo apt install git curl wget build-essential -y
```

## 🔧 Компиляция компонентов

### 1. ESP32 (основная платформа)

```bash
# Переход в директорию ESP32
cd esp32

# Установка зависимостей PlatformIO
pio lib install

# Компиляция проекта
pio run

# Загрузка в ESP32
pio run -t upload

# Мониторинг серийного порта
pio device monitor
```

### 2. STM32F407 (промышленная платформа)

```bash
# Переход в директорию STM32
cd stm32

# Установка зависимостей PlatformIO
pio lib install

# Компиляция проекта
pio run

# Загрузка в STM32F407
pio run -t upload

# Мониторинг серийного порта
pio device monitor
```

### 3. Node.js компоненты

```bash
# Переход в директорию Node.js
cd nodejs

# Установка зависимостей
npm install

# Компиляция (если есть TypeScript)
npm run build

# Запуск всех сервисов
npm start

# Или запуск отдельных сервисов:
npm run bridge      # CAN Bridge
npm run dashboard   # Web Dashboard
npm run api         # API Server
```

### 4. ROS 2 пакеты

```bash
# Переход в директорию ROS 2
cd ros2

# Установка зависимостей
pip install -r requirements.txt

# Компиляция ROS 2 пакетов
colcon build

# Запуск ROS 2
source install/setup.bash
ros2 launch care_ld2450_driver care.launch.py
```

## 🚀 Использование системы

### Полный запуск C.A.R.E.:

```bash
# 1. Запуск Node.js сервисов
cd nodejs
npm start &
NODE_PID=$!

# 2. Запуск ROS 2
cd ../ros2
source install/setup.bash
ros2 launch care_ld2450_driver care.launch.py &
ROS_PID=$!

# 3. Настройка CAN интерфейса
sudo ip link set can0 up type can bitrate 500000

# 4. Мониторинг системы
echo "C.A.R.E. система запущена!"
echo "Web Dashboard: http://localhost:3000"
echo "API Server: http://localhost:3001"
echo "ROS 2: ros2 topic list"
```

### Остановка системы:

```bash
# Остановка Node.js
kill $NODE_PID

# Остановка ROS 2
kill $ROS_PID

# Остановка CAN интерфейса
sudo ip link set can0 down
```

## 🔍 Проверка работоспособности

### 1. Проверка ESP32/STM32:

```bash
# Мониторинг серийного порта
pio device monitor

# Ожидаемый вывод:
# C.A.R.E. ESP32 Radar Module Starting...
# WiFi AP started: CARE_Radar
# LD2450 Radar initialized with UART2
# CAN interface initialized (TWAI)
# C.A.R.E. ESP32 Ready!
```

### 2. Проверка Node.js сервисов:

```bash
# Проверка Web Dashboard
curl http://localhost:3000/health

# Проверка API Server
curl http://localhost:3001/health

# Проверка CAN Bridge
# (должен быть виден в логах Node.js)
```

### 3. Проверка ROS 2:

```bash
# Список топиков
ros2 topic list

# Ожидаемые топики:
# /care_ld2450_driver/RadarTargets
# /care_ld2450_driver/SafetyStatus

# Просмотр данных
ros2 topic echo /care_ld2450_driver/RadarTargets
ros2 topic echo /care_ld2450_driver/SafetyStatus
```

### 4. Проверка CAN интерфейса:

```bash
# Просмотр CAN сообщений
candump can0

# Отправка тестового сообщения
cansend can0 100#01
```

## 🛠️ Отладка и решение проблем

### Проблемы с компиляцией ESP32:

```bash
# Очистка кэша PlatformIO
pio system prune

# Переустановка зависимостей
pio lib install --force

# Проверка конфигурации
pio run -t checkprogsize
```

### Проблемы с Node.js:

```bash
# Очистка node_modules
rm -rf node_modules package-lock.json
npm install

# Проверка версии Node.js
node --version
npm --version
```

### Проблемы с ROS 2:

```bash
# Переустановка зависимостей
pip install -r requirements.txt

# Перекомпиляция пакетов
colcon build --packages-select care_ld2450_driver
```

### Проблемы с CAN:

```bash
# Проверка CAN интерфейса
ip link show can0

# Перезапуск CAN интерфейса
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
```

## 📊 Мониторинг системы

### 1. Web Dashboard:
- Откройте http://localhost:3000
- Real-time визуализация радарных целей
- Статус безопасности
- Статистика системы

### 2. API Endpoints:
- `GET /api/status` - Статус системы
- `GET /api/radar/data` - Радарные данные
- `GET /api/safety/logs` - Логи безопасности
- `GET /api/statistics` - Статистика

### 3. ROS 2 топики:
- `/care_ld2450_driver/RadarTargets` - Радарные цели
- `/care_ld2450_driver/SafetyStatus` - Статус безопасности

## 🔧 Конфигурация

### Настройка радара:
1. Подключите LD2450 к ESP32/STM32
2. Используйте HLK-LD2450_TOOL для настройки
3. Установите параметры безопасности
4. Сохраните конфигурацию

### Настройка CAN:
1. Подключите CAN-трансивер
2. Настройте bitrate (500kbps)
3. Проверьте подключение

### Настройка WiFi (ESP32):
1. Подключитесь к AP "CARE_Radar"
2. Откройте http://192.168.4.1
3. Настройте параметры безопасности
4. Сохраните конфигурацию

## 🚨 Экстренная остановка

### Программная остановка:
```bash
# Остановка через ROS 2
ros2 service call /care_ld2450_driver/EmergencyStop std_srvs/srv/SetBool "{data: true}"

# Остановка через API
curl -X POST http://localhost:3001/api/safety/emergency-stop
```

### Аппаратная остановка:
- Нажмите кнопку Emergency Stop на роботе
- Отключите питание системы
- Используйте физический выключатель

## 📚 Дополнительные ресурсы

- **[INSTALL.md](INSTALL.md)** - Подробная установка
- **[DEVELOPMENT.md](DEVELOPMENT.md)** - Разработка
- **[docs/](docs/)** - Техническая документация
- **[nodejs/README.md](nodejs/README.md)** - Node.js документация
