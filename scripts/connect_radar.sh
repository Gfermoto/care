#!/bin/bash
# =============================================================================
# ESPHome LD2450 - Radar USB Connection Script for WSL
# =============================================================================
# Автоматически подключает радар к WSL через usbipd
# Поддерживает UART to USB переходники (CH340, CP2102, FT232)
#
# Использование: ./scripts/connect_radar.sh [BUSID]
# Пример: ./scripts/connect_radar.sh 5-6
# BUSID через переменную: RADAR_BUSID=3-4 ./scripts/connect_radar.sh
#
# Требования:
# - Windows с установленным usbipd-win
# - WSL2
# - Радар подключен к USB порту Windows
# =============================================================================

# BUSID: аргумент > переменная RADAR_BUSID > значение по умолчанию
BUSID=${1:-${RADAR_BUSID:-"5-6"}}

echo -e "\033[32m📡 Подключение радара в WSL...\033[0m"

# Выполняем usbipd команды через PowerShell
powershell.exe -Command "
    Write-Host '📋 Проверяем список USB устройств...' -ForegroundColor Cyan
    usbipd list

    Write-Host \"🔗 Подключаем радар (BUSID ${BUSID})...\" -ForegroundColor Yellow

    # Проверяем статус устройства
    \$status = usbipd list | Select-String '${BUSID}' | Select-String 'Shared'
    if (\$status) {
        Write-Host '✅ Радар уже привязан' -ForegroundColor Green
    } else {
        Write-Host '🔗 Привязываем радар...' -ForegroundColor Yellow
        usbipd bind --busid ${BUSID}
    }

    Write-Host '🔗 Подключаем радар к WSL...' -ForegroundColor Yellow
    usbipd attach --wsl --busid ${BUSID}

    Write-Host '✅ Радар подключен к WSL!' -ForegroundColor Green
"

echo ""
echo -e "\033[33m⏳ Ждем 10 секунд для выполнения команд в PowerShell...\033[0m"
sleep 10

echo -e "\033[35m🔍 Проверяем статус usbipd и доступность порта...\033[0m"
echo -e "\033[36m📋 Текущий статус USB устройств:\033[0m"
powershell.exe usbipd list

echo ""
echo -e "\033[35m🔍 Проверяем доступность порта...\033[0m"
if [ -e /dev/ttyUSB0 ]; then
    RADAR_PORT="/dev/ttyUSB0"
elif [ -e /dev/ttyUSB1 ]; then
    RADAR_PORT="/dev/ttyUSB1"
fi
if [ -n "$RADAR_PORT" ]; then
    echo -e "\033[32m✅ Порт $RADAR_PORT доступен!\033[0m"
    echo ""
    echo -e "\033[36m🚀 Доступные команды для работы с радаром:\033[0m"
    echo -e "\033[37m  # Монитор радара (Python - рекомендуется):\033[0m"
    if [ "$RADAR_PORT" = "/dev/ttyUSB0" ]; then
        echo -e "\033[37m  python3 scripts/radar.py\033[0m"
    else
        echo -e "\033[37m  python3 scripts/radar.py -p $RADAR_PORT\033[0m"
    fi
    echo ""
    echo -e "\033[37m  # Проверить все доступные порты:\033[0m"
    echo -e "\033[37m  ls -la /dev/ttyUSB*\033[0m"
else
    echo -e "\033[31m❌ Порты /dev/ttyUSB0 и /dev/ttyUSB1 не найдены\033[0m"
    echo -e "\033[33m🔧 Возможные решения:\033[0m"
    echo -e "\033[37m  1. Проверить все USB порты: ls -la /dev/ttyUSB*\033[0m"
    echo -e "\033[37m  2. Проверить usbipd статус: usbipd list (в PowerShell)\033[0m"
    echo -e "\033[37m  3. Переподключить USB кабель радара\033[0m"
    echo -e "\033[37m  4. Проверить питание радара\033[0m"
    echo -e "\033[37m  5. Попробовать другой USB порт на компьютере\033[0m"
    echo -e "\033[37m  6. Убедитесь, что устройство правильно определено в usbipd\033[0m"
    echo ""
    echo -e "\033[36m🔧 Ручной способ подключения:\033[0m"
    echo -e "\033[37m  1. Откройте PowerShell от имени администратора\033[0m"
    echo -e "\033[37m  2. Найдите BUSID вашего устройства: usbipd list\033[0m"
    echo -e "\033[37m  3. Привяжите устройство: usbipd bind --busid <BUSID>\033[0m"
    echo -e "\033[37m  4. Подключите к WSL: usbipd attach --wsl --busid <BUSID>\033[0m"
    echo ""
    echo -e "\033[37m  Или используйте скрипт с параметром:\033[0m"
    echo -e "\033[37m  ./scripts/connect_radar.sh <BUSID>\033[0m"
fi
