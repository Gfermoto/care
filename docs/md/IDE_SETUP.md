# Настройка IDE для разработки C.A.R.E.

## Проблема с импортами ROS2

При разработке Python кода для ROS2 в проекте C.A.R.E. могут возникать ошибки импорта типа:
- `Import "rclpy" could not be resolved`
- `Import "std_msgs.msg" could not be resolved`
- `Import "care_common.msg" could not be resolved`

Это происходит потому, что IDE не знает о путях к ROS2 модулям.

## Решение

### 1. Автоматическая настройка (рекомендуется)

Проект уже содержит правильные конфигурации:

- **pyrightconfig.json** - конфигурация для basedpyright
- **.vscode/settings.json** - настройки VS Code
- **scripts/verify_ros2_imports.py** - скрипт проверки импортов

### 2. Ручная настройка

#### VS Code / Cursor

1. Откройте настройки VS Code (Ctrl+,)
2. Найдите "python analysis extraPaths"
3. Добавьте следующие пути:
   ```
   /opt/ros/jazzy/lib/python3.12/site-packages
   /home/gfer/CARE/services/ros2/install/lib/python3.12/site-packages
   /home/gfer/CARE/services/ros2/install/lib/care_can_bridge_node
   /home/gfer/CARE/services/ros2/install/lib/care_common
   ```

#### PyCharm

1. File → Settings → Project → Python Interpreter
2. Нажмите на шестеренку → Show All
3. Выберите интерпретатор → Show Paths
4. Добавьте пути к ROS2 модулям

### 3. Проверка настройки

Запустите скрипт проверки:
```bash
cd /home/gfer/CARE
source /opt/ros/jazzy/setup.bash
source services/ros2/install/setup.bash
python3 scripts/verify_ros2_imports.py
```

Если все импорты работают, вы увидите:
```
🎉 Все импорты работают корректно!
```

### 4. Перезапуск IDE

После изменения конфигурации перезапустите IDE для применения настроек.

## Структура путей

```
/opt/ros/jazzy/lib/python3.12/site-packages/     # Системные ROS2 модули
├── rclpy/                                       # ROS2 Python API
├── std_msgs/                                    # Стандартные сообщения
└── ...

/home/gfer/CARE/services/ros2/install/           # Собранные модули проекта
├── lib/python3.12/site-packages/
│   └── care_common/                             # Общие сообщения C.A.R.E.
└── lib/care_can_bridge_node/
    └── can_bridge.py                            # Скрипт моста
```

## Устранение неполадок

### Ошибка "module not found"
1. Убедитесь, что ROS2 установлен: `ros2 --version`
2. Проверьте, что проект собран: `colcon build --merge-install`
3. Перезапустите IDE

### Ошибка "Import could not be resolved"
1. Проверьте, что pyrightconfig.json находится в корне проекта
2. Убедитесь, что пути в конфигурации корректны
3. Перезапустите языковой сервер (Ctrl+Shift+P → "Python: Restart Language Server")

### Ошибка форматирования Flake8
Импорты должны быть разделены пустыми строками:
```python
# Локальные импорты
from care_common.msg import RadarTarget

# ROS2 импорты  
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, String
```
