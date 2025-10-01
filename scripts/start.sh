#!/bin/bash

echo "🚀 C.A.R.E. - Запуск системы"
echo "============================"

# Переход в директорию проекта
cd /home/gfer/CARE

# Проверка конфигурации датчика
if [ -z "$CARE_SENSOR_TYPE" ]; then
    echo "⚠️ Тип датчика не настроен!"
    echo "🔧 Настройте датчик:"
    echo "   ./scripts/configure_sensor.sh"
    exit 1
fi

echo "📊 Конфигурация датчика: $CARE_SENSOR_TYPE"

# Проверка ROS 2 Jazzy
if [ ! -f "/opt/ros/jazzy/bin/ros2" ]; then
    echo "❌ ROS 2 Jazzy не найден!"
    echo "📦 Установите ROS 2 Jazzy:"
    echo "   chmod +x scripts/install_ros2_jazzy.sh"
    echo "   ./scripts/install_ros2_jazzy.sh"
    exit 1
fi

# Настройка ROS 2 окружения
echo "🌍 Настройка ROS 2 Jazzy..."
set +u  # Отключаем проверку неопределенных переменных
source /opt/ros/jazzy/setup.bash

# Проверка Node.js
if ! command -v node &> /dev/null; then
    echo "📦 Установка Node.js..."
    curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
    sudo apt-get install -y nodejs
fi

# Переход в nodejs
cd services/nodejs

# Очистка кэша npm
echo "🧹 Очистка кэша npm..."
npm cache clean --force

# Удаление старых node_modules
if [ -d "node_modules" ]; then
    echo "🗑️ Удаление старых зависимостей..."
    rm -rf node_modules
    rm -f package-lock.json
fi

# Выбор package.json в зависимости от типа датчика
if [ "$CARE_SENSOR_TYPE" = "mock" ]; then
    echo "🎯 Использование Mock CAN конфигурации..."
    cp package-mock.json package.json
else
    echo "📡 Использование реального CAN конфигурации..."
    # Восстанавливаем оригинальный package.json
    git checkout package.json 2>/dev/null || true
fi

# Установка зависимостей
echo "📦 Установка зависимостей..."
npm install

# Проверка установки rclnodejs только для реального CAN
if [ "$CARE_SENSOR_TYPE" = "real" ] && [ ! -d "node_modules/rclnodejs" ]; then
    echo "❌ rclnodejs не установлен!"
    echo "🔧 Попробуйте установить вручную:"
    echo "   npm install rclnodejs --build-from-source"
    exit 1
fi

# Запуск
echo "🚀 Запуск C.A.R.E. Node.js сервисов..."
echo "🌐 Web Dashboard: http://localhost:3000"
echo "🔌 API Server: http://localhost:3001"
echo "📡 CAN Bridge: подключение к ROS 2 Jazzy"
echo "🎯 Mock CAN: имитация LD2450 радара"
echo ""

npm start
