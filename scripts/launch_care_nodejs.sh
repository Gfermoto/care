#!/bin/bash

# C.A.R.E. Node.js Interface Launch Script
# Запускает Mock CAN → Web Dashboard → API Server

set -e

echo "🚀 Запуск C.A.R.E. Node.js Interface с Mock данными"
echo "=================================================="

# Проверка Node.js
if ! command -v node &> /dev/null; then
    echo "❌ Node.js не установлен. Установите Node.js 18+"
    exit 1
fi

NODE_VERSION=$(node --version | cut -d'v' -f2 | cut -d'.' -f1)
if [ "$NODE_VERSION" -lt 18 ]; then
    echo "❌ Требуется Node.js 18+. Текущая версия: $(node --version)"
    exit 1
fi

echo "✅ Node.js $(node --version) найден"

# Переход в директорию Node.js
cd /home/gfer/CARE/services/nodejs

# Проверка зависимостей
if [ ! -d "node_modules" ]; then
    echo "📦 Установка зависимостей..."
    npm install
fi

echo ""
echo "🎯 Запуск компонентов C.A.R.E. Node.js:"
echo "  1. Mock CAN Interface (генерация данных)"
echo "  2. Web Dashboard (http://localhost:3000)"
echo "  3. API Server (http://localhost:3001)"
echo ""

# Функция для очистки процессов
cleanup() {
    echo ""
    echo "🛑 Остановка C.A.R.E. Node.js Interface..."
    kill $MOCK_CAN_PID 2>/dev/null || true
    kill $DASHBOARD_PID 2>/dev/null || true
    kill $API_PID 2>/dev/null || true
    exit 0
}

# Обработка сигналов
trap cleanup SIGINT SIGTERM

# Запуск Mock CAN Interface в фоне
echo "✅ Starting Mock CAN Interface..."
node mock-can/index.js > /tmp/mock_can.log 2>&1 &
MOCK_CAN_PID=$!
sleep 2

# Запуск Web Dashboard в фоне
echo "✅ Starting Web Dashboard..."
node care-dashboard/index.js > /tmp/dashboard.log 2>&1 &
DASHBOARD_PID=$!
sleep 2

# Запуск API Server в фоне
echo "✅ Starting API Server..."
node care-api/index.js > /tmp/api.log 2>&1 &
API_PID=$!
sleep 2

echo ""
echo "🎉 C.A.R.E. Node.js Interface запущен!"
echo ""
echo "📊 Веб-интерфейсы:"
echo "  🌐 Web Dashboard: http://localhost:3000"
echo "  🔌 API Server: http://localhost:3001/health"
echo ""
echo "📡 Mock данные:"
echo "  🎯 CAN сообщения генерируются каждые 100мс"
echo "  📊 3 движущиеся цели с реалистичной физикой"
echo "  🚨 Emergency stop при приближении < 500mm"
echo ""
echo "🔍 Проверка статуса:"
echo "  curl http://localhost:3000/health"
echo "  curl http://localhost:3001/health"
echo "  curl http://localhost:3001/api/statistics"
echo ""
echo "🛑 Нажмите Ctrl+C для остановки"

# Ожидание завершения
wait $MOCK_CAN_PID $DASHBOARD_PID $API_PID
