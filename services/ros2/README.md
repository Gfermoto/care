# C.A.R.E. ROS2 Workspace

## 📊 Архитектура

Минимальный и изящный набор пакетов:

```
services/ros2/
├── care_common/                  # Общие сообщения/сервисы
├── care_can_bridge_node/        # 🚌 Прозрачный мост CAN ↔ ROS2
├── care_demo_node/              # 🎭 Демо/визуализация + настройка (0x400/0x401)
└── care_safety_controller_node/ # 🛡️ Расширенная логика безопасности
```

- Все ноды поддерживают режимы: `mode:=mock` или `mode:=real`
- Демо-нода умеет отправлять конфиг в контроллер/радар через CAN (`0x400/0x401`), ждать ACK
- Мост публикует ACK в `/care/config_ack_raw`

## 🚀 Запуск

```bash
# Сборка
cd /home/gfer/CARE/services/ros2
colcon build && source install/setup.bash

# Мост (real CAN)
ros2 run care_can_bridge_node can_bridge.py --ros-args -p mode:=real -p can_interface:=can0

# Демо (визуализация + конфиг через CAN)
ros2 run care_demo_node demo_node.py --ros-args -p mode:=mock -p can_interface:=can0

# Безопасность
ros2 run care_safety_controller_node safety_node.py --ros-args -p mode:=mock
```

## 🖼️ RViz

- Маркеры целей: `/care/rviz/targets`
- Статус: `/care/rviz/status`
- Fixed Frame: `care_radar`

## ⚙️ Конфигурация через CAN (из демо-ноды)

```bash
# SET (контроллер): bitrate=500k, mode=normal, SAVE
ros2 service call /care/configure_controller care_common/srv/ConfigController \
  "{device_id: 1, config_section: 'can', config_data: '{\"bitrate\": 500000, \"mode\": \"normal\"}', apply_immediately: true, save_to_flash: true}"

# SET (радар): max_targets=3, range=8м, update=20 Гц
ros2 service call /care/configure_controller care_common/srv/ConfigController \
  "{device_id: 1, config_section: 'radar', config_data: '{\"max_targets\": 3, \"range_m\": 8, \"update_hz\": 20}', apply_immediately: true, save_to_flash: true}"

# GET (контроллер)
ros2 service call /care/configure_controller care_common/srv/ConfigController \
  "{device_id: 1, config_section: 'can', config_data: '{\"__cmd__\": \"get\"}', apply_immediately: true, save_to_flash: false}"

# RESET (радар)
ros2 service call /care/configure_controller care_common/srv/ConfigController \
  "{device_id: 1, config_section: 'radar', config_data: '{\"__cmd__\": \"reset\"}', apply_immediately: true, save_to_flash: false}"
```

ACK/ERROR публикуются мостом в `/care/config_ack_raw`.

## 📡 CAN протокол (актуальный)
- `0x100` — Emergency Stop
- `0x200-0x202` — Target Data
- `0x300` — System Status
- `0x400` — Config Controller (SET/GET/SAVE/RESET)
- `0x401` — Config Radar (SET/GET/SAVE/RESET)
- `0x480` — ACK контроллера
- `0x481` — ACK радара

Подробнее: `docs/CAN_MESSAGES.md`.
