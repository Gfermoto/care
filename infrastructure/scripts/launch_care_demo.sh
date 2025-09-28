#!/bin/bash

# C.A.R.E. Demo Launch Script
# Запускает Mock датчик → ROS2 → RViz2

set -e

echo "🚀 Запуск C.A.R.E. Demo с RViz2 визуализацией"
echo "=============================================="

# Проверка ROS2 окружения
if [ -z "$ROS_DISTRO" ]; then
    echo "📡 Загрузка ROS2 окружения..."
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
fi

# Проверка сборки пакета
if [ ! -f "/home/gfer/CARE/services/ros2/install/care_radar_publisher/lib/care_radar_publisher/care_radar_node" ]; then
    echo "🔧 Сборка ROS2 пакета..."
    cd /home/gfer/CARE/services/ros2
    colcon build --packages-select care_radar_publisher
fi

# Загрузка workspace
echo "📦 Загрузка C.A.R.E. workspace..."
cd /home/gfer/CARE/services/ros2
set +u
source install/setup.bash
set -u

echo ""
echo "🎯 Запуск компонентов C.A.R.E.:"
echo "  1. ROS2 нода с Mock данными радара"
echo "  2. RViz2 с красивой конфигурацией"
echo ""

# Функция для очистки процессов
cleanup() {
    echo ""
    echo "🛑 Остановка C.A.R.E. Demo..."
    kill $ROS_PID 2>/dev/null || true
    kill $RVIZ_PID 2>/dev/null || true
    exit 0
}

# Обработка сигналов
trap cleanup SIGINT SIGTERM

# Запуск ROS2 ноды в фоне
echo "🎯 Запуск ROS2 ноды care_radar_publisher..."
ros2 run care_radar_publisher care_radar_node &
ROS_PID=$!

# Ждем запуска ноды
sleep 3

# Проверяем топики
echo "📊 Проверка топиков:"
timeout 5 ros2 topic list | grep care || echo "⚠️  Топики C.A.R.E. пока не активны"

# Запуск RViz2 с конфигурацией
echo "🎨 Запуск RViz2 с конфигурацией C.A.R.E...."
sleep 1

# Путь к конфигурации
RVIZ_CONFIG="/home/gfer/CARE/services/ros2/care_radar_publisher/config/care_radar.rviz"

if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" &
    RVIZ_PID=$!
else
    echo "⚠️  Конфигурация RViz2 не найдена, запуск с настройками по умолчанию..."
    rviz2 &
    RVIZ_PID=$!
fi

echo ""
echo "✅ C.A.R.E. Demo запущен!"
echo "📡 ROS2 нода: PID $ROS_PID"
echo "🎨 RViz2: PID $RVIZ_PID"
echo ""
echo "🎯 Топики для мониторинга:"
echo "  • ros2 topic echo /care/radar_targets"
echo "  • ros2 topic echo /care/safety_zone"  
echo "  • ros2 topic echo /care/status"
echo ""
echo "🎮 Для остановки нажмите Ctrl+C"
echo ""

# Ждем завершения процессов
wait $ROS_PID $RVIZ_PID
