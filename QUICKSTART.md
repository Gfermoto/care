# C.A.R.E. - Краткая инструкция

## 🎯 Демонстрация

### ROS2 Demo (3D визуализация)
```bash
./scripts/launch_care_demo.sh
```

### Node.js Demo (веб-интерфейс)
```bash
./scripts/launch_nodejs_demo.sh
```

**Результат:** RViz2 с анимированными целями, FOV и зоной безопасности

---

## 🔨 Компиляция

### ESP32
```bash
cd platforms/esp32
pio run
```

### STM32
```bash
cd platforms/stm32
pio run
```

### ROS2
```bash
cd services/ros2
source /opt/ros/jazzy/setup.bash
colcon build --merge-install
```

---

## 📊 Проверка демо

```bash
# Активные ноды
ros2 node list

# Топики
ros2 topic list | grep care

# Частота обновления
ros2 topic hz /care/radar_targets

# TF трансформации
ros2 run tf2_ros tf2_echo map radar_link
```

---

## 🛑 Остановка

```bash
killall -9 rviz2
pkill -f "can_bridge|care_demo"
```

---

## 📚 Документация

- **Демо**: [docs/md/DEMO_LAUNCH.md](docs/md/DEMO_LAUNCH.md)
- **Архитектура**: [docs/md/ARCHITECTURE.md](docs/md/ARCHITECTURE.md)  
- **Troubleshooting**: [docs/md/TROUBLESHOOTING.md](docs/md/TROUBLESHOOTING.md)
- **CAN протокол**: [docs/md/CAN_MESSAGES.md](docs/md/CAN_MESSAGES.md)

---

**C.A.R.E. System** - Collision Avoidance Radar Emergency

