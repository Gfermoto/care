# 📁 Структура проекта C.A.R.E.

## Корневая структура

```
CARE/
├── 📚 docs/                    # Документация
├── 🏗️ infrastructure/          # Docker и конфигурация
├── 🤖 platforms/               # Микроконтроллеры (ESP32, STM32)
├── 📜 scripts/                 # Все скрипты (установка, запуск, отладка)
├── 🌐 services/                # Сервисы (Node.js, ROS2)
├── 🧪 tests/                   # Тесты (unit, integration, e2e)
├── 🔧 .devcontainer/           # Dev Container для разработки
├── ⚙️ .github/                 # CI/CD, templates, workflows
├── 📄 .editorconfig            # Настройки редактора
├── 📄 .gitattributes           # Git атрибуты (EOL, бинарники)
├── 📄 .gitignore               # Игнорируемые файлы
├── 📄 .clang-format            # Форматирование C/C++
├── 📄 .cspell.json             # Словарь для проверки орфографии
├── 📄 CONTRIBUTING.md          # Руководство контрибьютора
├── 📄 LICENSE                  # MIT License
├── 📄 README.md                # Главный README
├── 📄 SECURITY.md              # Политика безопасности
└── 📄 PROJECT_STRUCTURE.md     # Этот файл
```

## Детальная структура

### 📚 docs/ - Документация
```
docs/
├── api/                        # API документация
├── architecture/               # Архитектурные документы
├── datasheets/                 # Даташиты радара LD2450
├── deployment/                 # Развёртывание
├── md/                         # Разные MD документы (legacy)
├── ARCHITECTURE.md             # Текущая архитектура
├── ARCHITECTURE_TARGET.md      # Целевая архитектура
├── CAN_ID_REFERENCE.md         # CAN ID справочник
├── CAN_MESSAGES.md             # CAN протокол
├── CONTROLLER_RADAR_SETUP.md   # Настройка контроллера/радара
└── DEPLOYMENT.md               # Развёртывание
```

### 🏗️ infrastructure/ - Инфраструктура
```
infrastructure/
├── configs/                    # Конфиги (requirements.txt)
└── docker/                     # Docker конфигурация
    ├── nodejs/                 # Dockerfile для Node.js
    ├── ros2/                   # Dockerfile для ROS2
    ├── docker-compose.yml      # Единый compose
    └── env.example             # Пример переменных окружения
```

### 📜 scripts/ - Скрипты
```
scripts/
├── configure_sensor.sh         # Конфигурация датчика
├── debug_com_port.py           # Отладка COM-порта
├── debug_web_interface.html    # Веб-отладка
├── install_ros2_dev.sh         # ROS2 dev установка
├── install_ros2_jazzy.sh       # ROS2 Jazzy установка
├── launch_care_demo.sh         # Запуск демо
├── monitor_care.sh             # Мониторинг
├── ros2_build.sh               # Сборка ROS2 пакетов
├── ros2_test.sh                # Запуск ROS2 тестов
├── start.sh                    # Основной запуск
├── test_mock_can.sh            # Тест mock CAN
└── uart_engineering_setup.sh   # UART инженерные команды
```

### 🤖 platforms/ - Микроконтроллеры
```
platforms/
├── esp32/                      # ESP32 платформа
│   ├── src/                    # Исходники
│   ├── include/                # Заголовки
│   ├── lib/                    # Библиотеки
│   ├── platformio.ini          # PlatformIO конфиг
│   └── sdkconfig.*             # ESP-IDF конфиг
└── stm32/                      # STM32 платформа
    ├── Src/                    # Исходники
    ├── Inc/                    # Заголовки
    └── platformio.ini          # PlatformIO конфиг
```

### 🌐 services/ - Сервисы
```
services/
├── nodejs/                     # Node.js сервисы
│   ├── care-api/               # REST API
│   ├── care-can-bridge/        # CAN ↔ WebSocket мост
│   ├── care-dashboard/         # Веб-интерфейс
│   ├── config/                 # Конфигурации
│   ├── mock-can/               # Mock CAN для разработки
│   ├── stm32-config/           # Конфигуратор STM32
│   ├── Dockerfile              # Dockerfile (legacy, дубль)
│   ├── package.json            # npm конфигурация
│   └── eslint.config.mjs       # ESLint конфиг
└── ros2/                       # ROS2 пакеты
    ├── care_common/            # Общие сообщения/сервисы
    ├── care_can_bridge_node/   # CAN ↔ ROS2 мост
    ├── care_demo_node/         # Демо + RViz конфиг
    ├── care_safety_controller_node/ # Логика безопасности
    └── third_party/            # Git submodules (iwr6843aop_pub)
```

### 🧪 tests/ - Тесты
```
tests/
├── unit/                       # Юнит-тесты
│   ├── nodejs/                 # Node.js модули
│   ├── ros2/                   # ROS2 пакеты
│   └── platforms/              # ESP32/STM32
├── integration/                # Интеграционные тесты
│   ├── nodejs/                 # Node.js + зависимости
│   ├── ros2/                   # ROS2 взаимодействие
│   └── platforms/              # Микроконтроллер + CAN
├── e2e/                        # End-to-end тесты
│   ├── nodejs/                 # Полный веб-стек
│   ├── ros2/                   # Полный ROS2 стек
│   └── platforms/              # Система с железом
├── pytest.ini                  # pytest конфиг
└── README.md                   # Описание тестов
```


### 🔧 .devcontainer/ - Dev Container
```
.devcontainer/
├── Dockerfile                  # ROS2 Humble Desktop + Node 18
└── devcontainer.json           # VS Code/Cursor конфиг
```

### ⚙️ .github/ - CI/CD
```
.github/
├── workflows/
│   ├── ci.yml                  # Основной CI (lint, build, test)
│   └── codeql.yml              # CodeQL анализ (C++/JS)
├── ISSUE_TEMPLATE/             # Шаблоны Issues
│   ├── bug_report.md           # Баг-репорт
│   └── feature_request.md      # Feature request
├── pull_request_template.md    # Шаблон PR
├── CODEOWNERS                  # Code owners
└── dependabot.yml              # Dependabot конфиг
```

## Ключевые файлы

| Файл | Назначение |
|------|------------|
| `.editorconfig` | Единые настройки отступов/EOL |
| `.gitattributes` | Нормализация EOL, бинарники |
| `.gitignore` | Исключения из Git |
| `.clang-format` | Форматирование C/C++ (Google style) |
| `.cspell.json` | Словарь для проверки орфографии |
| `CONTRIBUTING.md` | Руководство по контрибьюции |
| `SECURITY.md` | Политика безопасности |

## Принципы организации

✅ **Централизация Docker** - один `docker-compose.yml` в `infrastructure/docker/`  
✅ **Разделение concerns** - `services/`, `platforms/`, `infrastructure/`  
✅ **Тесты отдельно** - `tests/` с unit/integration/e2e  
✅ **Скрипты в одном месте** - `scripts/` для быстрого доступа  
✅ **CI/CD настроен** - `.github/workflows/` с линтами и тестами  
✅ **Dev Container** - `.devcontainer/` для одинаковой среды разработки  

## Улучшения (✅ выполнено)

- ✅ Централизован Docker (один compose)
- ✅ Добавлен `.editorconfig`
- ✅ Добавлен `.gitattributes`
- ✅ Добавлены Issue/PR templates
- ✅ Настроен CodeQL
- ✅ Добавлен `SECURITY.md`
- ✅ Добавлен `CODEOWNERS`
- ✅ Создана структура `tests/`
- ✅ Упрощена архитектура (убран nginx/redis)
- ✅ Удалён скомпрометированный ключ из истории

