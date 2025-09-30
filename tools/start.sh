#!/bin/bash

echo "🚀 C.A.R.E. - Быстрый запуск"
echo "============================"

# Переход в директорию проекта
cd /home/gfer/CARE

# Проверка конфигурации датчика
if [ -z "$CARE_SENSOR_TYPE" ]; then
    echo "⚠️ Тип датчика не настроен!"
    echo "🔧 Настройте датчик:"
    echo "   ./infrastructure/scripts/configure_sensor.sh"
    exit 1
fi

echo "📊 Конфигурация датчика: $CARE_SENSOR_TYPE"

# Проверка ROS 2 (только для STM32)
if [ "$CARE_SENSOR_TYPE" = "real" ]; then
    if [ ! -f "/opt/ros/jazzy/bin/ros2" ]; then
        echo "❌ ROS 2 не найден! Установите:"
        echo "   ./infrastructure/scripts/install_ros2_jazzy.sh"
        exit 1
    fi
    # Настройка ROS 2 окружения
    set +u
    source /opt/ros/jazzy/setup.bash
fi

# Запуск C.A.R.E.
echo "🚀 Запуск C.A.R.E. системы..."
./infrastructure/scripts/start.sh