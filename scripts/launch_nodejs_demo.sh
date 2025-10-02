#!/bin/bash

# C.A.R.E. Node.js Demo Launch Script
# Запускает интегрированную демонстрацию Node.js интерфейса

set -e

echo "🚀 Запуск C.A.R.E. Node.js Demo с Mock данными"
echo "=============================================="

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
echo "🎯 C.A.R.E. Node.js Demo включает:"
echo "  🎯 Mock CAN Interface - генерация радарных данных"
echo "  🌐 Web Dashboard - веб-интерфейс реального времени"
echo "  🔌 API Server - REST API для данных"
echo "  📊 Интеграция всех компонентов в одном процессе"
echo ""

# Функция для очистки процессов
cleanup() {
    echo ""
    echo "🛑 Остановка C.A.R.E. Node.js Demo..."
    pkill -f "node.*integrated-demo" 2>/dev/null || true
    exit 0
}

# Обработка сигналов
trap cleanup SIGINT SIGTERM

# Запуск интегрированной демонстрации
echo "✅ Starting C.A.R.E. Integrated Node.js Demo..."
node integrated-demo.js

echo ""
echo "🎉 C.A.R.E. Node.js Demo завершена!"
