#!/bin/bash

# C.A.R.E. Debug Script - пошаговая диагностика

set -e

echo "🔍 C.A.R.E. Debug Mode"
echo "===================="

# Загрузка ROS2 окружения
if [ -z "$ROS_DISTRO" ]; then
    echo "📡 Загрузка ROS2 окружения..."
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
fi

# Загрузка workspace
cd /home/gfer/CARE/services/ros2
if [ -f "install/setup.bash" ]; then
    set +u
    source install/setup.bash
    set -u
fi

echo ""
echo "1. 🔧 Сборка пакета..."
colcon build --packages-select care_radar_publisher

echo ""
echo "2. 📡 Запуск ROS2 ноды..."
ros2 run care_radar_publisher care_radar_node &
NODE_PID=$!

sleep 3

echo ""
echo "3. 📊 Проверка топиков..."
ros2 topic list | grep care

echo ""
echo "4. 🔄 Проверка TF..."
timeout 5 ros2 run tf2_tools view_frames.py || echo "TF проблемы обнаружены"

echo ""
echo "5. 📈 Проверка данных..."
timeout 3 ros2 topic echo /care/status --once || echo "Нет данных в /care/status"

echo ""
echo "6. 🎯 Проверка целей..."
timeout 3 ros2 topic echo /care/radar_targets --once || echo "Нет данных в /care/radar_targets"

echo ""
echo "7. 🎨 Запуск RViz2 в отладочном режиме..."
rviz2 -d /home/gfer/CARE/services/ros2/care_radar_publisher/config/care_radar.rviz &
RVIZ_PID=$!

echo ""
echo "✅ Debug запущен!"
echo "📡 ROS2 нода: PID $NODE_PID"
echo "🎨 RViz2: PID $RVIZ_PID"

# Функция очистки
cleanup() {
    echo ""
    echo "🛑 Остановка debug..."
    kill $NODE_PID 2>/dev/null || true
    kill $RVIZ_PID 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM

echo ""
echo "🎮 Для остановки нажмите Ctrl+C"
wait $NODE_PID $RVIZ_PID
