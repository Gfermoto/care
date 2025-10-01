#!/usr/bin/env python3
"""
Скрипт для проверки корректности импортов ROS2 модулей в проекте C.A.R.E.
"""
import sys
import os

def test_imports():
    """Тестирует все необходимые импорты ROS2."""
    print("🔍 Проверка импортов ROS2 модулей...")

    try:
        # Основные ROS2 модули
        import rclpy
        print("✅ rclpy импортирован успешно")

        from rclpy.node import Node
        print("✅ rclpy.node импортирован успешно")

        from std_msgs.msg import Header, String
        print("✅ std_msgs.msg импортирован успешно")

        # Модули проекта C.A.R.E.
        from care_common.msg import RadarTarget, RadarTargets, SystemStatus
        print("✅ care_common.msg импортирован успешно")

        print("\n🎉 Все импорты работают корректно!")
        return True

    except ImportError as e:
        print(f"❌ Ошибка импорта: {e}")
        return False
    except Exception as e:
        print(f"❌ Неожиданная ошибка: {e}")
        return False

def main():
    """Основная функция."""
    # Добавляем пути к ROS2 модулям
    ros2_paths = [
        "/opt/ros/jazzy/lib/python3.12/site-packages",
        "/home/gfer/CARE/services/ros2/install/lib/python3.12/site-packages",
        "/home/gfer/CARE/services/ros2/install/lib/care_can_bridge_node",
        "/home/gfer/CARE/services/ros2/install/lib/care_common"
    ]

    for path in ros2_paths:
        if os.path.exists(path):
            sys.path.insert(0, path)
            print(f"📁 Добавлен путь: {path}")

    success = test_imports()
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
