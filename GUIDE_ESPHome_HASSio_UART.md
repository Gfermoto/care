# ESPHome, Home Assistant и USB-UART для LD2450

Руководство по ESPHome, карточкам HA и настройке радара через USB-UART.

## Компоненты

1. ESPHome — прошивка для ESP32
2. Home Assistant — карточки и визуализация
3. USB-UART — настройка радара до подключения к ESP

---

## ESPHome

### Файлы

```
ESPHome/
├── LD2450_with_native_component.yaml
├── Plotly_Graph_Native_Component.txt
├── README.md
└── secrets.yaml.example
```

### LD2450_with_native_component.yaml

Нативный компонент LD2450, 3 зоны, 3 цели (X, Y, Speed, Angle), веб-интерфейс.

### ESP-IDF

UART 256000 стабильнее на ESP-IDF. Прошивка меньше, RAM меньше. ESPHome переходит на ESP-IDF по умолчанию.

### Быстрый старт

#### 1. Настройка секретов

```bash
cd ESPHome/
cp secrets.yaml.example secrets.yaml
nano secrets.yaml
```

Заполните:
```yaml
wifi_ssid: "YOUR_WIFI_SSID"
wifi_password: "YOUR_WIFI_PASSWORD"
ota_password: "YOUR_OTA_PASSWORD"
hotspot_password: "YOUR_HOTSPOT_PASSWORD"

# Генерация ключа: esphome encryption-key
api_encryption_key: "YOUR_API_ENCRYPTION_KEY"
```

#### 2. Компиляция и прошивка

```bash
cd ESPHome/
esphome compile LD2450_with_native_component.yaml
esphome upload LD2450_with_native_component.yaml
```

#### 3. Проверка работы

```bash
esphome logs LD2450_with_native_component.yaml
```

В логах: Target count: 3, Multi-Target mode: ON, данные X, Y.

#### 4. Home Assistant

Settings → Devices & Services → ld2450 → Configure → api_encryption_key из secrets.yaml.

### Ключевые компоненты

#### UART (256000 baud)
```yaml
uart:
  id: uart_bus
  tx_pin: GPIO17  # TX → RX радара
  rx_pin: GPIO16  # RX ← TX радара
  baud_rate: 256000
  parity: NONE
  stop_bits: 1
  data_bits: 8
```

#### Нативный компонент LD2450
```yaml
ld2450:
  id: ld2450_radar
  uart_id: uart_bus
```

#### Переключатель режима множественных целей
```yaml
switch:
  - platform: ld2450
    multi_target:
      name: "Режим нескольких целей"
      id: multi_target_mode
      restore_mode: RESTORE_DEFAULT_ON  # Автоматически ВКЛ
```

#### Аппаратные зоны радара (3 зоны)
```yaml
number:
  - platform: ld2450
    zone_1:
      x1: { name: "Зона 1 - X1" }
      y1: { name: "Зона 1 - Y1" }
      x2: { name: "Зона 1 - X2" }
      y2: { name: "Зона 1 - Y2" }
    presence_timeout: { name: "Зона 1 - Таймаут" }
  # ... аналогично для zone_2 и zone_3
```

**Дефолтные зоны:**
- **Зона 1:** 2м × 0.75м (глубина 250-1000мм)
- **Зона 2:** 4м × 2м (глубина 1000-3000мм)
- **Зона 3:** 6м × 3м (глубина 3000-6000мм)

### Сенсоры в HA

**Цели:**
- `sensor.ld2450_tsel_1_x` - Цель 1 - X
- `sensor.ld2450_tsel_1_y` - Цель 1 - Y
- `sensor.ld2450_tsel_1_speed` - Цель 1 - Скорость
- `sensor.ld2450_tsel_1_angle` - Цель 1 - Угол
- `sensor.ld2450_tsel_2_x` - Цель 2 - X
- `sensor.ld2450_tsel_2_y` - Цель 2 - Y
- `sensor.ld2450_tsel_2_speed` - Цель 2 - Скорость
- `sensor.ld2450_tsel_2_angle` - Цель 2 - Угол
- `sensor.ld2450_tsel_3_x` - Цель 3 - X
- `sensor.ld2450_tsel_3_y` - Цель 3 - Y
- `sensor.ld2450_tsel_3_speed` - Цель 3 - Скорость
- `sensor.ld2450_tsel_3_angle` - Цель 3 - Угол

**Аппаратные зоны:**
- `number.ld2450_zona_1_x1` - Зона 1 - X1
- `number.ld2450_zona_1_y1` - Зона 1 - Y1
- `number.ld2450_zona_1_x2` - Зона 1 - X2
- `number.ld2450_zona_1_y2` - Зона 1 - Y2
- `number.ld2450_zona_1_presence_timeout` - Зона 1 - Таймаут
- (аналогично для zone_2 и zone_3)

**Управление:**
- `switch.ld2450_rezhim_neskolkih_tselei` - Режим нескольких целей
- `button.ld2450_apply_default_zones` - Применить зоны по умолчанию
- `button.ld2450_radar_restart` - Перезагрузка радара

---

## Home Assistant карточки

### Plotly Graph Card

HACS → Frontend → Explore & Download → Plotly Graph Card. Перезагрузить HA.

https://github.com/dbuezas/lovelace-plotly-graph-card

### Добавление карточки

1. Откройте Dashboard → **Edit** → **Add Card**
2. Выберите **Manual** внизу
3. Скопируйте содержимое файла:

**Для нативного компонента:**
```
ESPHome/Plotly_Graph_Native_Component.txt
```

4. Вставьте в редактор
5. Если изменили `entity_name` в конфигурации, замените `ld2450` на своё
6. Сохраните

### Содержимое карточки

Цели — точки (синие/оранжевые/зелёные), координаты в мм. Зоны — полупрозрачные прямоугольники. Coverage — серая пунктирная линия, ~6м по центру.

### Настройка

**Основные параметры в `Plotly_Graph_Native_Component.txt`:**

```yaml
type: custom:plotly-graph
title: "LD2450 - Native Component"
refresh_interval: 2  # Обновление каждые 2 секунды
hours_to_show: current_day
```

**Изменение refresh_interval:**
- Для экономии ресурсов: увеличьте до 5-10 секунд
- Для быстрого обновления: уменьшите до 1 секунды

**Изменение масштаба:**
```yaml
xaxis:
  range: [-4000, 4000]  # -4м до +4м
yaxis:
  range: [8000, 0]      # 0м до 8м
```

### Структура

**Цели:**
```yaml
entities:
  - entity: ''
    name: Target1
    x:
      - $ex hass.states["sensor.ld2450_tsel_1_x"].state * -1
    'y':
      - $ex hass.states["sensor.ld2450_tsel_1_y"].state
```

**Зоны:**
```yaml
entities:
  - entity: ''
    name: HW Zone1
    mode: lines
    fill: toself
    fillcolor: RGBA(0,250,0,0.2)
    x:
      - $ex hass.states["number.ld2450_zona_1_x1"].state*1
      - $ex hass.states["number.ld2450_zona_1_x2"].state*1
      # ... и так далее для прямоугольника
```

---

## USB-UART настройка

### Подключение

USB-UART (CH340, CP2102, FT232):
```
LD2450                    USB-UART переходник
─────────────────         ───────────────────
VCC (5V)  ────────────►   5V
GND       ────────────►   GND
TX        ────────────►   RX (порт компьютера)
RX        ────────────►   TX (порт компьютера)
```

Не подключать VCC если радар питается от ESP32. Радар 5V 500mA. UART 3.3V.

### Скрипты

#### Подключение в WSL

```bash
# Скрипт автоматического подключения
./scripts/connect_radar.sh
```

Требует usbipd-win, WSL2.

#### Мониторинг
```bash
python3 scripts/radar.py
```

Выводит X, Y в мм. Протокол AA FF. pip install pyserial.

#### HLK-LD2450_TOOL

HLK-LD2450_TOOL_English.exe — официальный инструмент Hi-Link. LD2450 использует бинарный протокол, не AT.

**Как использовать:**
1. Подключите радар к COM порту Windows
2. Откройте `HLK-LD2450_TOOL_English.exe`
3. Выберите COM порт
4. Настройте параметры:
   - Зоны (Zone 1-3)
   - Чувствительность
   - Режим работы
   - Частота обновления
5. Нажмите "Write" для сохранения в радар

**Конфигурация сохраняется в радаре и будет загружена автоматически при подключении к ESP32!**

### Бинарный протокол

**Пакет от радара:**
```
Заголовок: AA FF
Разделитель: 55 CC
Данные: 3 цели × 8 байт каждая
```

**Формат команды (к радару):**
```
Header: FD FC FB FA
Length: 2 байта (little-endian)
Command: 2 байта
Data: N байт
Footer: 04 03 02 01
```

**Примеры команд (из ESPHome конфигурации):**

**1. Установка чувствительности (0x64):**
```cpp
uint8_t cmd_sensitivity[] = {
  0xFD, 0xFC, 0xFB, 0xFA,  // Header
  0x0A, 0x00,              // Length = 10 bytes
  0x64, 0x00,              // Command: Set sensitivity
  sens, sens, sens, sens, sens, sens, sens, sens,  // 8 диапазонов
  0x04, 0x03, 0x02, 0x01   // Footer
};
```

**2. Установка порога энергии (0x60):**
```cpp
uint8_t cmd_energy[] = {
  0xFD, 0xFC, 0xFB, 0xFA,  // Header
  0x04, 0x00,              // Length = 4 bytes
  0x60, 0x00,              // Command: Set energy threshold
  (uint8_t)(energy & 0xFF),        // Low byte
  (uint8_t)((energy >> 8) & 0xFF), // High byte
  0x04, 0x03, 0x02, 0x01   // Footer
};
```

**3. Сохранение конфигурации (0x60 0x61):**
```cpp
uint8_t cmd_save[] = {
  0xFD, 0xFC, 0xFB, 0xFA,
  0x02, 0x00,
  0x60, 0x61,              // Command: Save config
  0x04, 0x03, 0x02, 0x01
};
```

### Настройка через ESPHome/HA

Settings → Devices → ld2450 → Configure

**Кнопки:**
- Settings → Devices → ld2450 → Configure

Применить зоны, Записать конфигурацию, Применить и перезагрузить.

**Параметры:**
- **Чувствительность радара** (0-9)
- **Порог энергии** (100-10000)
- **Время удержания цели** (0-60 сек)
- **Фильтр скорости** (0-100 см/с)
- **Режим работы** (Normal/High Sensitivity/Low Power/High Precision)
- **Частота обновления** (1-20 Гц, макс 10 Гц для Multi-Target)

Зоны: X1, Y1, X2, Y2, таймаут. После изменений — «Применить и перезагрузить».

---

## Ссылки

- https://esphome.io/components/sensor/ld2450.html
- https://github.com/53l3cu5/ESP32_LD2450
- https://github.com/dbuezas/lovelace-plotly-graph-card
- https://53l3cu5.github.io

---

## FAQ

### ESPHome

**Почему ESP-IDF?** UART стабильнее, прошивка меньше. ESPHome переходит на ESP-IDF.

**Радар не в HA** — api_encryption_key в secrets.yaml и HA должен совпадать. Перезагрузить ESP32.

**Режим 3 целей** — включится сам или вручную в HA.

**Радар перезагружается** — питание 5V 500mA, нормальный USB.

**Plotly не отображается** — HACS, Plotly Graph Card, перезагрузка HA.

**Цели не на карточке** — имена entity в карточке совпадают с ESPHome?

**Порт /dev/ttyUSB0** — WSL: connect_radar.sh. Linux: lsusb, dmesg.

**Нет данных** — 256000 baud, TX/RX перекрёстно.

**Сохранить настройки** — кнопка в HA или HLK-LD2450_TOOL.

