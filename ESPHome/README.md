# ESPHome для HLK-LD2450

Конфиг на ESP-IDF с нативным компонентом LD2450.

Файлы:
- LD2450_with_native_component.yaml — конфиг ESPHome
- Plotly_Graph_Native_Component.txt — карточка для HA
- secrets.yaml.example — шаблон

## Быстрый старт

```bash
cp secrets.yaml.example secrets.yaml
nano secrets.yaml   # wifi, ota, api_encryption_key (esphome encryption-key)

esphome compile LD2450_with_native_component.yaml
esphome upload LD2450_with_native_component.yaml
```

Карточка: HACS → Plotly Graph Card, скопировать Plotly_Graph_Native_Component.txt в Manual card.

## Подключение

```
LD2450 → ESP32
VCC → 5V, GND → GND
TX → GPIO16 (RX2), RX → GPIO17 (TX2)
```

## Зоны по умолчанию

Зона 1: 2м × 0.75м (250-1000мм)
Зона 2: 4м × 2м (1000-3000мм)
Зона 3: 6м × 3м (3000-6000мм)

Подробнее в корневом README и GUIDE.
