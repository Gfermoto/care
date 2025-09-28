# 🚀 C.A.R.E. - Быстрый старт

## ⚡ **ЗАПУСК ВСЕГО ОДНОЙ КОМАНДОЙ:**

```bash
cd /home/gfer/CARE
./start.sh
```

## 🎯 **Что произойдет:**

1. ✅ **Проверка ROS 2** - автоматическая проверка установки
2. ✅ **Запуск C.A.R.E.** - все сервисы запускаются
3. ✅ **Mock CAN** - имитация LD2450 радара
4. ✅ **Web Dashboard** - веб-интерфейс доступен

## 📊 **РЕЗУЛЬТАТ:**

- 🌐 **Web Dashboard**: http://localhost:3000
- 🔌 **API Server**: http://localhost:3001
- 📡 **CAN Bridge**: ROS 2 топики
- 🎯 **Mock CAN**: имитация радара LD2450

## 🔧 **Если ROS 2 не установлен:**

```bash
# Установка ROS 2 Jazzy
./scripts/install_ros2_jazzy.sh

# Затем запуск
./start.sh
```

## 🧪 **Тестирование Mock CAN:**

```bash
# Тесты Mock CAN интерфейса
./scripts/test_mock_can.sh
```

## 🎉 **ГОТОВО!**

**C.A.R.E. запущен и работает!** 🛡️

---

**Запускайте прямо сейчас!**