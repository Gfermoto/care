# ESPHome + Home Assistant для радара HLK-LD2450

Форк с рабочими конфигурациями для LD2450. Убрано лишнее, оставлено то, что реально работает.

[![ESPHome](https://img.shields.io/badge/ESPHome-2023.12+-blue)](https://esphome.io)
[![Home Assistant](https://img.shields.io/badge/Home%20Assistant-2023.12+-orange)](https://www.home-assistant.io)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

## О проекте

Конфигурации для миллиметрового радара HLK-LD2450 и интеграции в Home Assistant через ESPHome.

В репозитории:
- ESPHome конфиги (ESP-IDF)
- Нативный компонент LD2450
- Карточки Plotly для HA
- Скрипты для USB-UART
- Документация

## Структура

```
CARE/
├── ESPHome/
│   ├── LD2450_with_native_component.yaml
│   ├── Plotly_Graph_Native_Component.txt
│   ├── README.md
│   └── secrets.yaml.example
├── case_stl/
│   ├── Box.stl / Box_fixed.stl
│   ├── Lid.stl / Lid(1).stl ...
│   └── README.md
├── scripts/
│   ├── radar.py
│   ├── connect_radar.sh
│   └── README.md
├── GUIDE_ESPHome_HASSio_UART.md
├── README.md
└── LICENSE
```

## Галерея

![Устройство](device_photos/photo.jpg)

![HA Plotly Radar Card](device_photos/ha_radar.png)

## Быстрый старт

### 1. Клонирование

```bash
git clone <your-repo-url>
cd CARE
```

### 2. Секреты

```bash
cd ESPHome/
cp secrets.yaml.example secrets.yaml
nano secrets.yaml
```

Заполните wifi_ssid, wifi_password, ota_password, hotspot_password, api_encryption_key.

Ключ API: `esphome encryption-key`

### 3. Сборка и прошивка

```bash
cd ESPHome/
esphome compile LD2450_with_native_component.yaml
esphome upload LD2450_with_native_component.yaml
```

### 4. Home Assistant

Settings → Devices & Services → найти "ld2450" → Configure → ввести api_encryption_key из secrets.yaml.

### 5. Карточка Plotly

HACS → Plotly Graph Card. Dashboard → Edit → Add Card → Manual → вставить содержимое Plotly_Graph_Native_Component.txt.

## Документация

Подробнее: [GUIDE_ESPHome_HASSio_UART.md](GUIDE_ESPHome_HASSio_UART.md)

Ссылки:
- [ESPHome LD2450](https://esphome.io/components/sensor/ld2450.html)
- [Оригинальный проект](https://github.com/53l3cu5/ESP32_LD2450)
- [Plotly Graph Card](https://github.com/dbuezas/lovelace-plotly-graph-card)
- [Конфигуратор зон](https://53l3cu5.github.io)
- [Корпус STL](case_stl/README.md)

## Подключение

```
LD2450          ESP32
VCC (5V)   →    5V
GND        →    GND
TX         →    GPIO16 (RX2)
RX         →    GPIO17 (TX2)
```

Радар: 5V 500mA минимум. UART 3.3V, питание 5V.

| Параметр | Значение |
|----------|----------|
| Частота | 24 GHz |
| Дальность | 0.5 - 8 м |
| Угол обзора | ±60° гориз., ±45° верт. |
| Цели | до 3 |
| UART | 256000 baud |
| Питание | 5V DC, 200-500 mA |

## Возможности

ESPHome: 3 зоны, 3 цели (X, Y, Speed, Angle), OTA, LED по зонам.

Home Assistant: Plotly, настройка зон, автоматизации.

USB-UART: radar.py, connect_radar.sh.

## Скрипты

```bash
python3 scripts/radar.py
./scripts/connect_radar.sh
```

Настройка радара: HLK-LD2450_TOOL_English.exe (официальный инструмент Hi-Link).

## Требования

- ESPHome 2023.12+
- Home Assistant 2023.12+ (опционально)
- Python 3 + pyserial для скриптов (`pip install -r scripts/requirements.txt`)

Железо: ESP32, HLK-LD2450, USB-UART (опционально).

## ESP-IDF

UART на 256000 стабильнее, прошивка меньше (~17%), RAM меньше (~13%). ESPHome переходит на ESP-IDF по умолчанию. Конфиг: ESPHome/LD2450_with_native_component.yaml

## Лицензия

MIT. См. [LICENSE](LICENSE).

## Благодарности

53l3cu5 (ESP32_LD2450), dbuezas (Plotly Graph Card), Hi-Link (LD2450).

---

Версия 1.0, октябрь 2025
