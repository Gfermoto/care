# Тесты C.A.R.E.

Структура тестов проекта C.A.R.E.

## Структура

```
tests/
├── unit/           # Юнит-тесты (изолированные компоненты)
│   ├── nodejs/     # Node.js модули
│   ├── ros2/       # ROS2 пакеты
│   └── platforms/  # ESP32/STM32
├── integration/    # Интеграционные тесты (взаимодействие компонентов)
│   ├── nodejs/     # Node.js + Redis/CAN
│   ├── ros2/       # ROS2 nodes взаимодействие
│   └── platforms/  # Микроконтроллер + CAN
└── e2e/            # End-to-end (полная система)
    ├── nodejs/     # Веб-интерфейс
    ├── ros2/       # Полный ROS2 стек
    └── platforms/  # Система с железом
```

## Запуск тестов

### Node.js
```bash
cd services/nodejs
npm test
```

### ROS2
```bash
./scripts/ros2_test.sh
```

### Python (если есть)
```bash
pytest tests/unit/
pytest tests/integration/
```

## Требования к тестам

- [ ] Покрытие кода > 70%
- [ ] Все критические пути покрыты
- [ ] CI проверяет тесты при каждом PR
- [ ] Моки для внешних зависимостей

