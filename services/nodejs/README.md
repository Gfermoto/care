# 🟢 C.A.R.E. Node.js Components

Node.js компоненты для системы C.A.R.E. (Collaborative Awareness Radar for Empathic interaction).

## 📁 Структура

```
nodejs/
├── care-can-bridge/     # CAN ↔ ROS 2 мост
├── care-dashboard/      # Веб-дашборд
├── care-api/           # REST API сервер
├── package.json        # Зависимости
├── index.js           # Главный файл
└── README.md          # Документация
```

## 🚀 Быстрый старт

### Установка зависимостей

```bash
cd nodejs
npm install
```

### Запуск всех сервисов

```bash
npm start
```

### Запуск отдельных сервисов

```bash
# CAN Bridge
npm run bridge

# Web Dashboard
npm run dashboard

# API Server
npm run api
```

## 🔧 Компоненты

### 1. **CAN Bridge** (`care-can-bridge/`)

**Функции:**
- Мост между CAN и ROS 2
- Обработка радарных данных
- Управление экстренной остановкой
- Статистика производительности

**CAN ID:**
- `0x100` - Emergency Stop
- `0x200-0x202` - Radar Targets (0-2)
- `0x300` - System Status
- `0x400` - Configuration

### 2. **Web Dashboard** (`care-dashboard/`)

**Функции:**
- Реальное время мониторинг
- Визуализация радарных целей
- Статус безопасности
- WebSocket соединения

**Порт:** 3000
**URL:** http://localhost:3000

### 3. **API Server** (`care-api/`)

**Функции:**
- REST API для конфигурации
- Экспорт данных (JSON/CSV)
- Статистика системы
- Логирование безопасности

**Порт:** 3001
**Endpoints:**
- `GET /health` - Health check
- `GET /api/radar/data` - Радарные данные
- `POST /api/radar/data` - Запись данных
- `GET /api/safety/logs` - Логи безопасности
- `POST /api/safety/logs` - Запись логов
- `GET /api/config/safety` - Конфигурация
- `POST /api/config/safety` - Обновление конфигурации
- `GET /api/statistics` - Статистика
- `GET /api/export/radar` - Экспорт радарных данных
- `GET /api/export/safety` - Экспорт логов безопасности

## 📊 Архитектура

```
ESP32/STM32 → CAN → Node.js CAN Bridge → ROS 2
                    ↓
              Web Dashboard (Real-time)
                    ↓
              API Server (Data/Config)
```

## 🛠️ Зависимости

### Основные:
- **socketcan** - CAN интерфейс
- **rclnodejs** - ROS 2 интеграция
- **express** - Web сервер
- **ws** - WebSocket
- **csv-writer** - Экспорт данных

### Разработка:
- **nodemon** - Автоперезагрузка
- **jest** - Тестирование
- **supertest** - API тестирование

## 🔌 CAN Протокол

### Формат сообщений:

#### Emergency Stop (ID: 0x100)
```
Data[0] = 0x01 (Active) / 0x00 (Inactive)
```

#### Radar Target (ID: 0x200-0x202)
```
Data[0-1] = X coordinate (mm)
Data[2-3] = Y coordinate (mm)
Data[4-5] = Distance (mm)
Data[6-7] = Speed (mm/s)
```

#### System Status (ID: 0x300)
```
Data[0] = System status
Data[1] = Active targets count
Data[2-3] = Safety distance (mm)
```

## 🌐 Web Dashboard

### Функции:
- **Real-time radar display** - Визуализация целей
- **Safety status** - Статус безопасности
- **System statistics** - Статистика системы
- **Emergency alerts** - Аварийные уведомления

### Технологии:
- **HTML5 Canvas** - Радарная визуализация
- **WebSocket** - Real-time обновления
- **CSS3 Animations** - Анимации
- **Responsive Design** - Адаптивный дизайн

## 📈 API Endpoints

### Радарные данные:
```bash
# Получить данные
GET /api/radar/data?limit=100&offset=0

# Записать данные
POST /api/radar/data
{
  "target_id": 0,
  "x": 1000,
  "y": 500,
  "distance": 1118,
  "speed": 50,
  "valid": true
}
```

### Конфигурация безопасности:
```bash
# Получить конфигурацию
GET /api/config/safety

# Обновить конфигурацию
POST /api/config/safety
{
  "min_safe_distance": 500,
  "max_safe_distance": 5000,
  "safety_angle": 30,
  "emergency_stop_enabled": true
}
```

### Экспорт данных:
```bash
# JSON экспорт
GET /api/export/radar?format=json&date=2024-01-15

# CSV экспорт
GET /api/export/radar?format=csv&date=2024-01-15
```

## 🔧 Конфигурация

### Environment Variables:
```bash
# Порты сервисов
PORT=3000                    # Dashboard port
API_PORT=3001               # API port

# CAN настройки
CAN_INTERFACE=can0          # CAN interface
CAN_BITRATE=500000          # CAN bitrate

# ROS 2 настройки
ROS_DOMAIN_ID=0             # ROS 2 domain ID
```

## 🚀 Production Deployment

### PM2 (Process Manager):
```bash
# Установка PM2
npm install -g pm2

# Запуск с PM2
pm2 start index.js --name "care-nodejs"

# Мониторинг
pm2 monit
```

### Docker:
```dockerfile
FROM node:18-alpine
WORKDIR /app
COPY package*.json ./
RUN npm ci --only=production
COPY . .
EXPOSE 3000 3001
CMD ["node", "index.js"]
```

## 📝 Логирование

### Уровни логов:
- **INFO** - Общая информация
- **WARN** - Предупреждения
- **ERROR** - Ошибки
- **DEBUG** - Отладочная информация

### Файлы логов:
- `logs/care-bridge.log` - CAN Bridge логи
- `logs/care-dashboard.log` - Dashboard логи
- `logs/care-api.log` - API логи

## 🧪 Тестирование

```bash
# Запуск тестов
npm test

# Тестирование API
npm run test:api

# Тестирование CAN Bridge
npm run test:bridge
```

## 🔍 Мониторинг

### Health Checks:
- **Dashboard**: http://localhost:3000/health
- **API**: http://localhost:3001/health
- **CAN Bridge**: Статистика в консоли

### Метрики:
- Количество CAN сообщений
- Количество ROS 2 сообщений
- Время отклика
- Ошибки

## 🛡️ Безопасность

### Защита:
- **Helmet** - HTTP заголовки безопасности
- **CORS** - Cross-Origin Resource Sharing
- **Rate Limiting** - Ограничение запросов
- **Input Validation** - Валидация входных данных

## 📚 Документация

- **[CAN Bridge](care-can-bridge/README.md)** - Документация CAN моста
- **[Web Dashboard](care-dashboard/README.md)** - Документация дашборда
- **[API Server](care-api/README.md)** - Документация API

## 🤝 Вклад в проект

1. Fork репозитория
2. Создайте feature branch
3. Commit изменения
4. Push в branch
5. Создайте Pull Request

## 📄 Лицензия

MIT License - см. [LICENSE](LICENSE) файл.

---

**C.A.R.E.** — потому что безопасность начинается с **осознания присутствия живого рядом**.
