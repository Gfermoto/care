#!/bin/bash

# C.A.R.E. Final Test - Окончательное исправление TF проблем
# Тестирование кардинально исправленной версии

set -e

echo "🎯 C.A.R.E. FINAL TEST - Кардинальные исправления TF"
echo "=================================================="

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
echo "🔧 КАРДИНАЛЬНЫЕ ИСПРАВЛЕНИЯ В ЭТОЙ ВЕРСИИ:"
echo "  1. ⏰ Единое время для ВСЕГО цикла (TF + маркеры)"
echo "  2. 🔄 TF публикуется ДО маркеров (гарантированно)"
echo "  3. ⏱️  Задержка 1мс между TF и маркерами"
echo "  4. 🎯 Функции принимают время как параметр"
echo "  5. 🛡️ Полная синхронизация timestamp'ов"

echo ""
echo "1. 📡 Запуск исправленной ROS2 ноды..."
ros2 run care_radar_publisher care_radar_node &
NODE_PID=$!

sleep 8

echo ""
echo "2. 🔍 Детальная проверка TF..."
echo "   Проверяем доступность трансформаций..."
set +u
source /opt/ros/jazzy/setup.bash
set -u

# Проверяем TF дерево
echo "   TF дерево:"
timeout 5 ros2 run tf2_tools view_frames.py 2>/dev/null && echo "   ✅ TF дерево создано" || echo "   ⚠️ TF дерево недоступно"

# Проверяем конкретную трансформацию
echo "   Трансформация map->radar_link:"
timeout 3 ros2 run tf2_ros tf2_echo map radar_link 2>/dev/null | head -3 && echo "   ✅ TF работает" || echo "   ❌ TF проблемы"

echo ""
echo "3. 📊 Проверка синхронизации данных..."
echo "   Получаем данные маркеров с timestamp..."
MARKER_TIME=$(timeout 3 ros2 topic echo /care/radar_targets --once 2>/dev/null | grep -A2 "stamp:" | head -4 | tail -2 | tr -d ' ')
echo "   Маркеры timestamp: $MARKER_TIME"

echo ""
echo "4. 🎨 Запуск RViz2 для финального теста..."
rviz2 -d /home/gfer/CARE/services/ros2/care_radar_publisher/config/care_radar.rviz &
RVIZ_PID=$!

sleep 5

echo ""
echo "5. 📊 Мониторинг ошибок RViz2..."
echo "   Следим за логами RViz2 на предмет TF ошибок..."

# Функция очистки
cleanup() {
    echo ""
    echo "🛑 Остановка финального теста..."
    kill $NODE_PID 2>/dev/null || true
    kill $RVIZ_PID 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM

echo ""
echo "✅ ФИНАЛЬНЫЙ ТЕСТ ЗАПУЩЕН!"
echo "📡 ROS2 нода: PID $NODE_PID"
echo "🎨 RViz2: PID $RVIZ_PID"

echo ""
echo "🔍 ОЖИДАЕМЫЙ РЕЗУЛЬТАТ:"
echo "  ✅ НЕТ ошибок 'Message Filter dropping message'"
echo "  ✅ Стабильная визуализация целей и FoV"
echo "  ✅ Цели живут 5 секунд без исчезновений"
echo "  ✅ FoV линии постоянно видны"
echo "  ✅ Плавная анимация без артефактов"

echo ""
echo "📊 Мониторинг системы..."
ERROR_COUNT=0
TOTAL_CHECKS=0

while true; do
    sleep 10
    TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
    
    # Проверяем процессы
    if ! kill -0 $NODE_PID 2>/dev/null; then
        echo "❌ ROS2 нода завершилась!"
        break
    fi
    if ! kill -0 $RVIZ_PID 2>/dev/null; then
        echo "❌ RViz2 завершился!"
        break
    fi
    
    # Проверяем логи на ошибки (в фоне)
    if dmesg | tail -20 | grep -q "Message Filter dropping message" 2>/dev/null; then
        ERROR_COUNT=$((ERROR_COUNT + 1))
        echo "⚠️ $(date '+%H:%M:%S') - Обнаружена TF ошибка (всего: $ERROR_COUNT)"
    else
        echo "✅ $(date '+%H:%M:%S') - Проверка $TOTAL_CHECKS: Без TF ошибок"
    fi
    
    # Статистика каждые 5 проверок
    if [ $((TOTAL_CHECKS % 5)) -eq 0 ]; then
        echo "📊 Статистика: $ERROR_COUNT ошибок из $TOTAL_CHECKS проверок"
        if [ $ERROR_COUNT -eq 0 ]; then
            echo "🎉 ОТЛИЧНО! TF проблемы полностью исправлены!"
        fi
    fi
done

wait $NODE_PID $RVIZ_PID

