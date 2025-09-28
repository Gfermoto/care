#!/bin/bash

# C.A.R.E. Sensor Configuration Script
# Выбор между реальным датчиком и Mock CAN

echo "🔧 C.A.R.E. - Конфигурация датчика"
echo "=================================="

# Функция для выбора типа датчика
configure_sensor() {
    echo ""
    echo "Выберите тип датчика:"
    echo "1) 🎯 Mock CAN (для тестирования без железа)"
    echo "2) 📡 Реальный CAN (для работы с физическим радаром)"
    echo ""
    read -p "Введите номер (1-2): " choice
    
    case $choice in
        1)
            echo "🎯 Выбран Mock CAN датчик"
            export CARE_SENSOR_TYPE=mock
            echo "export CARE_SENSOR_TYPE=mock" >> ~/.bashrc
            echo "✅ Конфигурация сохранена в ~/.bashrc"
            ;;
        2)
            echo "📡 Выбран реальный CAN датчик"
            export CARE_SENSOR_TYPE=real
            echo "export CARE_SENSOR_TYPE=real" >> ~/.bashrc
            echo "✅ Конфигурация сохранена в ~/.bashrc"
            
            # Проверка CAN интерфейса
            echo ""
            echo "🔍 Проверка CAN интерфейса..."
            if ip link show can0 >/dev/null 2>&1; then
                echo "✅ CAN интерфейс can0 найден"
            else
                echo "⚠️ CAN интерфейс can0 не найден"
                echo "📋 Для настройки CAN выполните:"
                echo "   sudo ip link set can0 up type can bitrate 500000"
            fi
            ;;
        *)
            echo "❌ Неверный выбор!"
            exit 1
            ;;
    esac
}

# Проверка существующей конфигурации
if [ -n "$CARE_SENSOR_TYPE" ]; then
    echo "📊 Текущая конфигурация: $CARE_SENSOR_TYPE"
    echo ""
    read -p "Хотите изменить конфигурацию? (y/n): " change
    if [ "$change" = "y" ] || [ "$change" = "Y" ]; then
        configure_sensor
    fi
else
    configure_sensor
fi

echo ""
echo "🎉 Конфигурация завершена!"
echo "🔄 Перезагрузите терминал или выполните: source ~/.bashrc"
echo ""
echo "🚀 Для запуска C.A.R.E.: ./scripts/run_care.sh"
