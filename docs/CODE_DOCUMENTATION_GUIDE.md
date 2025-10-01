# 📝 Руководство по документированию кода C.A.R.E.

## Общие принципы

1. **Документируйте интерфейсы, а не реализацию**
2. **Код должен быть самодокументируемым** - используйте понятные имена
3. **Комментарии объясняют "почему", а не "что"**
4. **Все публичные API должны быть задокументированы**

---

## 🔷 C/C++ (ROS2 nodes, платформы)

### Doxygen стиль

#### Файлы
```cpp
/**
 * @file can_bridge_node.cpp
 * @brief CAN ↔ ROS2 мост для радара LD2450
 * @author Gfermoto
 * @date 2025-10-01
 * 
 * Подробное описание назначения файла.
 */
```

#### Классы
```cpp
/**
 * @brief Класс для управления CAN шиной
 * 
 * Этот класс обрабатывает сообщения CAN от радара LD2450
 * и преобразует их в ROS2 топики.
 * 
 * @details Поддерживает:
 * - Emergency Stop (0x100)
 * - Target Data (0x200-0x202)
 * - System Status (0x300)
 */
class CanBridge {
public:
    /**
     * @brief Конструктор CAN моста
     * @param node_name Имя ROS2 ноды
     * @param can_interface Имя CAN интерфейса (например, "can0")
     * @throws std::runtime_error если CAN интерфейс недоступен
     */
    CanBridge(const std::string& node_name, const std::string& can_interface);
    
    /**
     * @brief Обработка входящего CAN сообщения
     * @param[in] frame CAN фрейм
     * @return true если сообщение обработано успешно
     * @note Этот метод вызывается из callback'а CAN драйвера
     */
    bool processCanFrame(const can_frame& frame);
    
private:
    /**
     * @brief Публикация emergency stop события
     * @param[in] data Данные из CAN (8 байт)
     * 
     * @warning Этот метод должен быть real-time safe!
     * Данные публикуются в топик /care/emergency_stop
     */
    void publishEmergencyStop(const uint8_t* data);
};
```

#### Функции
```cpp
/**
 * @brief Вычисляет контрольную сумму CAN сообщения
 * @param[in] data Указатель на данные (8 байт)
 * @param[in] length Длина данных
 * @return Контрольная сумма (uint8_t)
 * 
 * @pre data != nullptr
 * @pre length <= 8
 * 
 * Алгоритм: XOR всех байтов данных
 */
uint8_t calculateChecksum(const uint8_t* data, size_t length);
```

#### Перечисления и константы
```cpp
/**
 * @brief CAN ID для различных типов сообщений
 */
enum class CanId : uint32_t {
    EMERGENCY_STOP = 0x100,  ///< Аварийная остановка
    TARGET_DATA_1  = 0x200,  ///< Данные цели #1 (x, y)
    TARGET_DATA_2  = 0x201,  ///< Данные цели #2 (vx, vy)
    SYSTEM_STATUS  = 0x300   ///< Статус системы
};

/** @brief Максимальное количество целей */
constexpr size_t MAX_TARGETS = 3;

/** @brief Период обновления датчика (мс) */
constexpr int SENSOR_UPDATE_RATE_MS = 50;
```

---

## 🟢 Node.js (JavaScript/TypeScript)

### JSDoc стиль

#### Модули
```javascript
/**
 * @module care-can-bridge
 * @description CAN ↔ WebSocket мост для C.A.R.E.
 * @author Gfermoto
 * @version 1.0.0
 */
```

#### Функции
```javascript
/**
 * Обрабатывает CAN сообщение и отправляет через WebSocket
 * @param {Object} canMessage - CAN сообщение
 * @param {number} canMessage.id - CAN ID (0x100-0x4FF)
 * @param {Buffer} canMessage.data - Данные (до 8 байт)
 * @param {WebSocket} ws - WebSocket соединение
 * @returns {Promise<boolean>} true если отправка успешна
 * @throws {Error} если WebSocket не готов
 * 
 * @example
 * const message = { id: 0x100, data: Buffer.from([0, 0, 0, 0, 0, 0, 0, 1]) };
 * await processCanMessage(message, wsConnection);
 */
async function processCanMessage(canMessage, ws) {
    // ...
}
```

#### Классы
```javascript
/**
 * Менеджер CAN соединений
 * @class
 */
class CanManager {
    /**
     * Создает новый менеджер CAN
     * @param {Object} config - Конфигурация
     * @param {string} config.interface - CAN интерфейс ("can0", "vcan0")
     * @param {boolean} [config.mockMode=false] - Режим эмуляции
     */
    constructor(config) {
        /** @private {string} */
        this.interface = config.interface;
        
        /** @private {boolean} */
        this.mockMode = config.mockMode || false;
    }
    
    /**
     * Отправляет CAN сообщение
     * @param {number} id - CAN ID
     * @param {Array<number>} data - Данные (до 8 элементов)
     * @returns {Promise<void>}
     */
    async send(id, data) {
        // ...
    }
}
```

#### Типы (TypeScript/JSDoc)
```javascript
/**
 * @typedef {Object} Target
 * @property {number} id - ID цели (0-2)
 * @property {number} x - Координата X (мм)
 * @property {number} y - Координата Y (мм)
 * @property {number} vx - Скорость X (мм/с)
 * @property {number} vy - Скорость Y (мм/с)
 * @property {number} distance - Расстояние (мм)
 * @property {boolean} dangerous - Опасная цель
 */

/**
 * @typedef {Object} RadarStatus
 * @property {boolean} ready - Радар готов
 * @property {number} targetCount - Количество целей
 * @property {Array<Target>} targets - Массив целей
 */
```

---

## 🐍 Python (скрипты)

### Docstrings (Google style)

```python
"""
Утилита для отладки COM порта радара LD2450.

Этот модуль предоставляет функции для:
- Чтения сырых данных с UART
- Парсинга протокола LD2450
- Визуализации данных

Example:
    $ python debug_com_port.py /dev/ttyUSB0
"""

def parse_ld2450_frame(data: bytes) -> dict:
    """
    Парсит фрейм протокола LD2450.
    
    Args:
        data: Сырые байты (должно быть 29+ байт)
        
    Returns:
        Словарь с полями:
        - targets: List[dict] - список целей
        - timestamp: int - метка времени
        
    Raises:
        ValueError: если данные некорректны
        
    Note:
        Протокол описан в docs/datasheets/LD2450 serial port communication protocol V1.03.pdf
        
    Example:
        >>> data = b'\xAA\xFF\x03\x00...'
        >>> frame = parse_ld2450_frame(data)
        >>> print(frame['targets'])
        [{'x': 1000, 'y': 500, 'speed': 100}]
    """
    pass

class RadarConnection:
    """
    Менеджер подключения к радару LD2450.
    
    Attributes:
        port (str): Имя COM порта
        baudrate (int): Скорость (256000 по умолчанию)
        timeout (float): Таймаут чтения (сек)
    """
    
    def __init__(self, port: str, baudrate: int = 256000):
        """
        Инициализирует подключение к радару.
        
        Args:
            port: Имя COM порта (например, '/dev/ttyUSB0')
            baudrate: Скорость UART (по умолчанию 256000)
            
        Raises:
            serial.SerialException: если порт недоступен
        """
        pass
```

---

## 📋 Требования по уровням

### Обязательно документировать:
- ✅ Все публичные API (классы, функции, методы)
- ✅ Параметры функций
- ✅ Возвращаемые значения
- ✅ Исключения/ошибки
- ✅ Файлы (краткое описание)

### Желательно документировать:
- ⚠️ Сложные алгоритмы (почему именно так)
- ⚠️ Хитрости и workaround'ы
- ⚠️ TODOs и FIXMEs
- ⚠️ Примеры использования

### Не документировать:
- ❌ Очевидные вещи (`i++; // инкремент i`)
- ❌ Приватные методы (если они тривиальны)
- ❌ Геттеры/сеттеры без логики

---

## 🔧 Генерация документации

### Doxygen (C/C++)
```bash
# Генерация HTML документации
doxygen Doxyfile

# Просмотр
xdg-open docs/api/html/index.html
```

### JSDoc (Node.js)
```bash
cd services/nodejs
npm run docs  # Генерирует docs/jsdoc/
```

### CI/CD
Документация автоматически генерируется при push в main и публикуется на GitHub Pages.

---

## 📚 Примеры из проекта

См. хорошо задокументированные файлы:
- `services/ros2/care_can_bridge_node/src/can_bridge.cpp`
- `services/nodejs/care-api/index.js`
- `scripts/debug_com_port.py`

---

## 🔍 Проверка качества

### Запуск линтеров с проверкой комментариев:
```bash
# C++: проверка Doxygen предупреждений
doxygen Doxyfile 2>&1 | grep "warning"

# Node.js: ESLint требует JSDoc
npm run lint

# Python: pydocstyle
pydocstyle scripts/
```

---

**Помните:** Хорошая документация экономит время всем, включая вас через 6 месяцев! 🚀

