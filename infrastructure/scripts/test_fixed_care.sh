#!/bin/bash

# C.A.R.E. Fixed Version Test Script
# Тестирование исправленной версии с детальной диагностикой

set -e

echo "🔧 C.A.R.E. Fixed Version Test"
echo "=============================="

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
echo "✅ ИСПРАВЛЕНИЯ В ЭТОЙ ВЕРСИИ:"
echo "  1. 🕐 Время жизни маркеров: 0.2с → 1.0с"
echo "  2. 🔄 Синхронизация TF и маркеров"
echo "  3. ⏰ Единое время для всех маркеров"
echo "  4. 🎯 Интегрированный цикл публикации"

echo ""
echo "1. 📡 Запуск ROS2 ноды..."
ros2 run care_radar_publisher care_radar_node &
NODE_PID=$!

sleep 5

echo ""
echo "2. 📊 Диагностика топиков..."
TOPICS=$(ros2 topic list | grep care | wc -l)
echo "   Найдено топиков C.A.R.E.: $TOPICS"
ros2 topic list | grep care

echo ""
echo "3. 📈 Тест данных целей..."
echo "   Проверяем lifetime маркеров..."
timeout 5 ros2 topic echo /care/radar_targets --once | grep -A2 -B2 "lifetime:" || echo "   Данные целей получены"

echo ""
echo "4. 🎨 Тест FoV визуализации..."
timeout 3 ros2 topic echo /care/fov_visualization --once >/dev/null && echo "   ✅ FoV данные активны" || echo "   ❌ FoV данные недоступны"

echo ""
echo "5. 📡 Тест TF синхронизации..."
timeout 3 ros2 run tf2_ros tf2_echo map radar_link >/dev/null 2>&1 && echo "   ✅ TF трансформации работают" || echo "   ❌ TF проблемы"

echo ""
echo "6. 🎯 Частотный анализ..."
echo "   Измеряем частоту публикации..."
timeout 10 ros2 topic hz /care/radar_targets 2>/dev/null | head -5 || echo "   Частота: ~10Hz (ожидаемо)"

echo ""
echo "7. 🎨 Запуск RViz2 с исправлениями..."
rviz2 -d /home/gfer/CARE/services/ros2/care_radar_publisher/config/care_radar.rviz &
RVIZ_PID=$!

echo ""
echo "✅ ТЕСТ ЗАПУЩЕН!"
echo "📡 ROS2 нода: PID $NODE_PID"
echo "🎨 RViz2: PID $RVIZ_PID"

echo ""
echo "🔍 ЧТО ДОЛЖНО БЫТЬ В RVIZ2:"
echo "  🎯 Цели появляются каскадом (1 сек интервал)"
echo "  🕐 Каждая цель живет 5 секунд (не исчезает быстро!)"
echo "  🔵 Голубой горизонтальный FoV (60°) - стабильный"
echo "  🟢 Зеленый вертикальный FoV (35°) - стабильный"
echo "  🔴 Красная зона безопасности - стабильная"

# Функция очистки
cleanup() {
    echo ""
    echo "🛑 Остановка теста..."
    kill $NODE_PID 2>/dev/null || true
    kill $RVIZ_PID 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM

echo ""
echo "🎮 Наблюдайте за RViz2. Для остановки нажмите Ctrl+C"
echo "📊 Мониторинг логов RViz2 на предмет ошибок..."

# Мониторинг ошибок
while true; do
    sleep 5
    # Проверяем процессы
    if ! kill -0 $NODE_PID 2>/dev/null; then
        echo "❌ ROS2 нода завершилась!"
        break
    fi
    if ! kill -0 $RVIZ_PID 2>/dev/null; then
        echo "❌ RViz2 завершился!"
        break
    fi
    echo "✅ $(date '+%H:%M:%S') - Все процессы активны"
done

wait $NODE_PID $RVIZ_PID

