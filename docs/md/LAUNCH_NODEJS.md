# 🚀 Запуск Node.js компонентов C.A.R.E.

## 📋 **Пошаговая инструкция**

### **1. Установка Node.js (если не установлен):**

```bash
# Обновление системы
sudo apt update

# Установка Node.js 18.x
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs

# Проверка установки
node --version
npm --version
```

### **2. Установка системных зависимостей:**

```bash
# Для socketcan (CAN интерфейс)
sudo apt-get install -y libsocketcan-dev build-essential

# CAN утилиты
sudo apt-get install -y can-utils

# Дополнительные инструменты
sudo apt-get install -y curl wget git
```

### **3. Переход в директорию Node.js:**

```bash
cd /home/gfer/CARE/nodejs
```

### **4. Установка зависимостей:**

```bash
npm install
```

### **5. Запуск Node.js сервисов:**

#### **Вариант A: Все сервисы сразу**
```bash
npm start
```

#### **Вариант B: Отдельные сервисы**
```bash
# Только Dashboard (http://localhost:3000)
npm run dashboard

# Только API (http://localhost:3001)
npm run api

# Только CAN Bridge (требует ROS 2)
npm run bridge
```

#### **Вариант C: Режим разработки**
```bash
npm run dev
```

## 🔧 **Настройка CAN интерфейса (если нужно):**

```bash
# Настройка CAN интерфейса
sudo ip link set can0 up type can bitrate 500000

# Проверка CAN
candump can0
```

## 📊 **Проверка работоспособности:**

### **1. Web Dashboard:**
- Откройте браузер: http://localhost:3000
- Должен появиться интерфейс C.A.R.E.

### **2. API Server:**
```bash
curl http://localhost:3001/health
```

### **3. Логи Node.js:**
```bash
# В консоли будут видны логи:
# [INFO] C.A.R.E. Node.js Services Starting...
# [INFO] Starting API Server...
# [INFO] Starting Web Dashboard...
# [SUCCESS] Web Dashboard running on http://localhost:3000
# [SUCCESS] API Server running on http://localhost:3001
```

## ⚠️ **Возможные проблемы и решения:**

### **Проблема 1: Node.js не установлен**
```bash
# Решение: установить Node.js
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs
```

### **Проблема 2: npm install не работает**
```bash
# Решение: очистить кэш и переустановить
npm cache clean --force
sudo npm install
```

### **Проблема 3: socketcan не устанавливается**
```bash
# Решение: установить системные библиотеки
sudo apt-get install -y libsocketcan-dev build-essential
npm install socketcan --build-from-source
```

### **Проблема 4: Порты заняты**
```bash
# Решение: освободить порты
sudo lsof -i :3000
sudo lsof -i :3001
sudo kill -9 <PID>
```

### **Проблема 5: rclnodejs не работает**
```bash
# Решение: ROS 2 не установлен
# CAN Bridge будет работать только с ROS 2
# Dashboard и API будут работать без ROS 2
```

## 🎯 **Ожидаемый результат:**

После успешного запуска вы увидите:

```
🚀 Starting C.A.R.E. Node.js Services...
📊 Starting API Server...
🌐 Starting Web Dashboard...
📡 Starting CAN Bridge...
✅ API Server running on http://localhost:3001
✅ Web Dashboard running on http://localhost:3000
✅ CAN Bridge connected to ROS 2
🎉 All C.A.R.E. Node.js services started successfully!
```

## 🔍 **Мониторинг:**

### **Логи в реальном времени:**
- В консоли будут видны все события
- CAN сообщения от ESP32/STM32
- WebSocket соединения
- API запросы

### **Веб-интерфейс:**
- **Dashboard**: http://localhost:3000
- **API**: http://localhost:3001/health
- **Статистика**: http://localhost:3001/api/statistics

## 🛑 **Остановка:**

```bash
# Остановка через Ctrl+C
# Или найти процесс и остановить
ps aux | grep node
kill <PID>
```

## 🎉 **Готово!**

**Node.js компоненты C.A.R.E. запущены и готовы к работе!**

- 🌐 **Web Dashboard** - мониторинг в реальном времени
- 🔌 **API Server** - конфигурация и данные
- 📡 **CAN Bridge** - связь с микроконтроллерами
- 🤖 **ROS 2** - интеграция с роботами

**C.A.R.E. работает!** 🛡️

