# Скрипты для LD2450

Мониторинг и подключение радара через USB-UART.

## connect_radar.sh

Подключает радар к WSL через usbipd.

```bash
./scripts/connect_radar.sh [BUSID]
RADAR_BUSID=3-4 ./scripts/connect_radar.sh
```

Нужно: Windows + usbipd-win, WSL2.

## radar.py

Читает координаты X,Y из потока радара (протокол AA FF).

```bash
python3 scripts/radar.py
python3 scripts/radar.py -p /dev/ttyUSB1
python3 scripts/radar.py -b 115200
```

Зависимости: `pip install -r scripts/requirements.txt`

## Проблемы

Порт не найден — проверить USB, запустить connect_radar.sh, смотреть `ls /dev/ttyUSB*`.

Нет данных — питание 5V, скорость 256000, TX/RX перекрёстно.

Permission denied: `sudo chmod 666 /dev/ttyUSB0` или добавить в группу dialout.
