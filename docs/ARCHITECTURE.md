# 🏗️ C.A.R.E. - Архитектура системы

## 📊 Обзор архитектуры

C.A.R.E. (Collision Avoidance Radar Emergency System) - это высокопроизводительная система предотвращения столкновений, построенная на основе микросервисной архитектуры с поддержкой двух платформ микроконтроллеров.

## 🎯 Архитектурные принципы

### 1. **Разделение ответственности**
- **Платформы**: ESP32 и STM32F407 работают независимо
- **Сервисы**: Node.js, ROS2, Nginx изолированы в контейнерах
- **Данные**: Четкое разделение конфигурации и рабочих данных

### 2. **Масштабируемость**
- **Микросервисы**: Каждый сервис может масштабироваться независимо
- **Контейнеризация**: Docker обеспечивает консистентное развертывание
- **Load Balancing**: Nginx для распределения нагрузки

### 3. **Надежность**
- **Health Checks**: Мониторинг состояния всех компонентов
- **Graceful Degradation**: Система продолжает работать при отказе компонентов
- **Data Persistence**: Redis для кэширования критических данных

## 🤖 Платформы микроконтроллеров

### ESP32 - Автономная система
```
┌─────────────────┐    ┌─────────────┐    ┌──────────────┐
│   LD2450 Radar │────│    ESP32    │────│ CAN Transceiver │
│                 │    │             │    │              │
│                 │    │ Web Server  │    │    (MCP2515) │
└─────────────────┘    │ WiFi AP     │    └──────────────┘
                       │ TWAI CAN    │           │
                       └─────────────┘           │
                               │                 │
                       ┌─────────────┐           │
                       │ Web Client  │           │
                       │192.168.4.1  │           │
                       └─────────────┘           │
                                                 │
                                        ┌────────▼────────┐
                                        │   ROS2 Host     │
                                        │   (can0)        │
                                        └─────────────────┘
```

**Характеристики:**
- **Автономность**: Полностью независимая работа
- **Веб-интерфейс**: Встроенный AsyncWebServer
- **CAN**: TWAI контроллер + внешний трансивер MCP2515
- **WiFi**: Access Point режим для конфигурации
- **Питание**: 5V, низкое энергопотребление

### STM32F407 - Высокопроизводительная система
```
┌─────────────────┐    ┌──────────────┐    ┌──────────────┐
│   LD2450 Radar │────│  STM32F407   │────│ Built-in CAN │
│                 │    │              │    │   Controller │
│                 │    │   168MHz     │    │   (bxCAN)    │
└─────────────────┘    │   1MB Flash  │    └──────────────┘
                       │   192KB RAM  │           │
                       └──────────────┘           │
                                                  │
                                         ┌────────▼────────┐
                                         │   ROS2 Host     │
                                         │   (can0)        │
                                         │                 │
                                         │  ┌─────────────┐│
                                         │  │  Node.js    ││
                                         │  │ Web Services││
                                         │  └─────────────┘│
                                         └─────────────────┘
```

**Характеристики:**
- **Производительность**: ARM Cortex-M4 168MHz
- **CAN**: Встроенный bxCAN контроллер
- **Память**: 1MB Flash, 192KB RAM
- **Интерфейсы**: UART для LD2450, CAN для ROS2
- **Веб-интерфейс**: Node.js сервисы на хосте

## 🌐 Сервисы хоста

### Node.js микросервисы (только для STM32)
```
┌─────────────────────────────────────────────────────────────┐
│                    Node.js Services                        │
├─────────────────┬─────────────────┬─────────────────────────┤
│  CAN Bridge     │   Dashboard     │       API Server       │
│                 │                 │                         │
│ • CAN ↔ ROS2    │ • Real-time UI  │ • REST API             │
│ • Data routing  │ • WebSocket     │ • Configuration        │
│ • Protocol conv │ • Visualization │ • Data export          │
└─────────────────┴─────────────────┴─────────────────────────┘
```

**Компоненты:**
- **care-can-bridge**: Мост между CAN и ROS2
- **care-dashboard**: Веб-интерфейс реального времени
- **care-api**: REST API для конфигурации
- **care-config**: Сервис управления конфигурацией

### ROS2 система
```
┌─────────────────────────────────────────────────────────────┐
│                      ROS2 Humble                           │
├─────────────────┬─────────────────┬─────────────────────────┤
│   Driver Node   │  Safety Node    │    Visualization       │
│                 │                 │                         │
│ • LD2450 driver │ • Safety zones  │ • RViz2                │
│ • Data parsing  │ • Emergency     │ • rqt tools            │
│ • CAN interface │ • Alerts        │ • Topic monitoring     │
└─────────────────┴─────────────────┴─────────────────────────┘
```

**Топики:**
- `/radar_targets`: Данные о целях
- `/safety_status`: Статус безопасности
- `/emergency_stop`: Аварийная остановка

**Сервисы:**
- `/set_safety_zone`: Настройка зон безопасности
- `/get_config`: Получение конфигурации
- `/reset_system`: Сброс системы

### Infrastructure сервисы
```
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│     Nginx       │  │     Redis       │  │    Docker       │
│                 │  │                 │  │                 │
│ • Reverse Proxy │  │ • Data Cache    │  │ • Orchestration │
│ • Load Balancer │  │ • Session Store │  │ • Service Mesh  │
│ • SSL/TLS       │  │ • Pub/Sub       │  │ • Health Checks │
└─────────────────┘  └─────────────────┘  └─────────────────┘
```

## 🔄 Потоки данных

### ESP32 поток данных
```
LD2450 → UART → ESP32 → Web Interface (192.168.4.1)
                  ↓
               TWAI CAN → MCP2515 → CAN Bus → ROS2 Host
```

### STM32 поток данных
```
LD2450 → UART → STM32F407 → bxCAN → CAN Bus → ROS2 Host
                                                  ↓
                                            Node.js Services
                                                  ↓
                                            Web Interface (localhost:3000)
```

## 🐳 Контейнеризация

### Development (на хосте)
```
Host Machine
├── ROS2 Jazzy (native)
├── Node.js Services (native)
├── Redis (Docker)
└── Nginx (Docker)
```

### Production (в контейнерах)
```
Docker Compose
├── care-ros2 (ROS2 Humble)
├── care-nodejs (Node.js Services)
├── care-redis (Redis Cache)
└── care-nginx (Reverse Proxy)
```

## 🔧 Конфигурация

### Environment Variables
```bash
# Sensor Type
CARE_SENSOR_TYPE=mock|real

# ROS2 Configuration
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# CAN Configuration
CAN_INTERFACE=can0
CAN_BITRATE=500000

# Node.js Configuration
NODE_ENV=production
API_PORT=3001
DASHBOARD_PORT=3000
```

### Configuration Files
```
infrastructure/configs/
├── ros2.yaml          # ROS2 parameters
├── nginx.conf         # Nginx configuration
├── redis.conf         # Redis configuration
└── sensor-config.js   # Sensor configuration
```

## 📊 Мониторинг и логирование

### Health Checks
- **ROS2**: `ros2 node list`
- **Node.js**: `curl http://localhost:3001/health`
- **Redis**: `redis-cli ping`
- **Nginx**: HTTP status codes

### Logging
```
logs/
├── ros2/              # ROS2 logs
├── nodejs/            # Node.js logs
├── nginx/             # Nginx access/error logs
└── system/            # System logs
```

## 🚀 Развертывание

### Quick Start
```bash
# 1. Конфигурация
./infrastructure/scripts/configure_sensor.sh

# 2. Установка зависимостей
./infrastructure/scripts/install_ros2_jazzy.sh

# 3. Запуск системы
./start.sh
```

### Production Deployment
```bash
# 1. Сборка контейнеров
docker-compose -f infrastructure/docker/docker-compose.yml build

# 2. Запуск в продакшене
docker-compose -f infrastructure/docker/docker-compose.yml up -d
```

## 🔍 Troubleshooting

### Общие проблемы
1. **CAN интерфейс не найден**: `sudo ip link set can0 up type can bitrate 500000`
2. **ROS2 не запускается**: Проверить `source /opt/ros/jazzy/setup.bash`
3. **Node.js сервисы недоступны**: Проверить порты 3000/3001
4. **Docker контейнеры не запускаются**: Проверить `docker logs <container>`

### Диагностические команды
```bash
# Проверка CAN
candump can0

# Проверка ROS2
ros2 topic list
ros2 node list

# Проверка Node.js
curl http://localhost:3001/health

# Проверка Docker
docker ps
docker logs care-nodejs
```

---

**Архитектура C.A.R.E. обеспечивает высокую производительность, надежность и масштабируемость для критически важных применений в робототехнике и системах безопасности.**
