#!/bin/bash

# C.A.R.E. System Monitor Script
# Мониторинг топиков и данных системы

set -e

echo "📊 C.A.R.E. System Monitor"
echo "=========================="

# Загрузка ROS2 окружения
if [ -z "$ROS_DISTRO" ]; then
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
echo "🎯 Активные топики C.A.R.E.:"
ros2 topic list | grep care || echo "❌ Нет активных топиков C.A.R.E."

echo ""
echo "📡 Информация о топиках:"
echo "------------------------"

for topic in "/care/status" "/care/radar_targets" "/care/safety_zone"; do
    if ros2 topic list | grep -q "$topic"; then
        echo "✅ $topic"
        ros2 topic info "$topic" 2>/dev/null || echo "   ⚠️  Информация недоступна"
    else
        echo "❌ $topic - не активен"
    fi
done

echo ""
echo "🔄 Мониторинг данных (Ctrl+C для остановки):"
echo "============================================"

# Функция мониторинга
monitor_topics() {
    while true; do
        echo ""
        echo "⏰ $(date '+%H:%M:%S')"
        echo "-------------------"
        
        # Статус системы
        if timeout 2 ros2 topic echo /care/status --once >/dev/null 2>&1; then
            STATUS=$(timeout 2 ros2 topic echo /care/status --once 2>/dev/null | grep "data:" | cut -d'"' -f2)
            echo "📊 Статус: $STATUS"
        else
            echo "📊 Статус: ❌ Недоступен"
        fi
        
        # Количество целей
        if timeout 2 ros2 topic echo /care/radar_targets --once >/dev/null 2>&1; then
            TARGETS=$(timeout 2 ros2 topic echo /care/radar_targets --once 2>/dev/null | grep -c "id:" || echo "0")
            echo "🎯 Цели: $TARGETS обнаружено"
        else
            echo "🎯 Цели: ❌ Данные недоступны"
        fi
        
        # Частота публикации
        if timeout 2 ros2 topic hz /care/status >/dev/null 2>&1; then
            echo "📈 Частота: Активна"
        else
            echo "📈 Частота: ❌ Нет публикации"
        fi
        
        sleep 3
    done
}

# Обработка сигналов
trap 'echo -e "\n🛑 Мониторинг остановлен"; exit 0' SIGINT SIGTERM

# Запуск мониторинга
monitor_topics

