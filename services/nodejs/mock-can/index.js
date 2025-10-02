#!/usr/bin/env node

/**
 * @file index.js
 * @module mock-can
 * @description C.A.R.E. Mock CAN Interface - Эмулятор CAN шины для тестирования
 * @author C.A.R.E. Development Team
 * @version 1.0.0
 *
 * @description
 * Эмулирует физический CAN интерфейс для разработки и тестирования без реального
 * оборудования (ESP32/STM32 + радар LD2450). Генерирует реалистичные данные целей
 * с движением, триггерит события emergency stop.
 *
 * ## Основные функции:
 * - Генерация синтетических CAN сообщений (0x100-0x400)
 * - Симуляция 1-3 движущихся целей
 * - Эмуляция emergency stop при приближении целей
 * - Реалистичная физика движения (скорость, траектория)
 * - EventEmitter API совместимый с socketcan
 *
 * ## Генерируемые CAN ID:
 * - `0x100` - Emergency Stop (при distance < safetyZone.minDistance)
 * - `0x200-0x202` - Target Data (x, y, distance, speed для 3 целей)
 * - `0x300` - System Status (состояние системы)
 *
 * ## События (EventEmitter):
 * - `message` - новое CAN сообщение получено
 * - `start` - Mock CAN запущен
 * - `stop` - Mock CAN остановлен
 *
 * @example
 * const MockCANInterface = require('./mock-can');
 *
 * const mockCAN = new MockCANInterface({
 *   targetCount: 2,
 *   updateInterval: 50, // мс
 *   safetyZone: { minDistance: 300, maxDistance: 6000 }
 * });
 *
 * mockCAN.on('message', (msg) => {
 *   console.log(`CAN ID: 0x${msg.id.toString(16)}, Data:`, msg.data);
 * });
 *
 * mockCAN.start();
 */

const EventEmitter = require('events');

// CAN Message IDs for C.A.R.E.
const CAN_IDS = {
    EMERGENCY_STOP: 0x100,
    TARGET_0: 0x200,
    TARGET_1: 0x201,
    TARGET_2: 0x202,
    STATUS: 0x300,
    CONFIG: 0x400
};

class MockCANInterface extends EventEmitter {
    constructor(options = {}) {
        super();

        this.options = {
            bitrate: 500000,
            channel: 'mock_can0',
            targetCount: 3,
            updateInterval: 100, // ms
            safetyZone: {
                minDistance: 500, // mm
                maxDistance: 6500, // mm - ограничиваем до 6.5м
                angleRange: 120 // degrees
            },
            ...options
        };

        this.isRunning = false;
        this.targets = [];
        this.emergencyStop = false;
        this.systemStatus = 0x01; // System OK

        // Статистика
        this.stats = {
            messagesSent: 0,
            startTime: Date.now()
        };

        // Синусоидальные параметры для движения
        this.time = 0;
        this.maxTargets = 3;
        this.minTargets = 1;

        this.initializeTargets();
    }

    initializeTargets() {
        // КРИТИЧНО: Очищаем массив целей перед инициализацией!
        this.targets = [];

        // Стабильное количество целей от 1 до 3
        const targetCount = Math.floor(Math.random() * 3) + 1; // 1, 2 или 3 цели
        console.log(`🎯 Initializing ${targetCount} targets with sinusoidal movement`);

        // Инициализация случайных целей СТРОГО в пределах FOV и 6.5м
        for (let i = 0; i < targetCount; i++) {
            // Простая инициализация (как в ROS)
            const target = {
                id: i,
                x: 0, // будет вычислено в updateTargetPosition
                y: 0, // будет вычислено в updateTargetPosition
                distance: 0, // будет вычислено в updateTargetPosition
                angle: 0, // будет вычислено в updateTargetPosition
                speed: 1500, // фиксированная скорость 1.5 м/с
                valid: true,
                lastUpdate: Date.now(),
                createdAt: Date.now() // время создания для расчета времени жизни
            };

            // Вычисляем начальную позицию
            this.updateTargetPosition(target);
            console.log(`🎯 Init Target ${i}: smooth movement, ${(target.distance/1000).toFixed(1)}m at ${target.angle.toFixed(1)}°`);
            this.targets.push(target);
        }

        console.log(`✅ Initialized ${this.targets.length} targets within 6.5m`);
    }

    // Обновление позиции цели с упрощённой синусоидальной моделью
    updateTargetPosition(target) {
        const currentTime = Date.now();

        // Инициализируем параметры движения при первом вызове
        if (!target.movementParams) {
            // Инициализируем позицию, если она не задана
            if (!target.x || !target.y) {
                const angle = Math.random() * Math.PI * 2;
                const dist = 2000 + Math.random() * 3000; // 2-5м
                target.x = Math.sin(angle) * dist;
                target.y = Math.cos(angle) * dist;
            }

            target.movementParams = {
                // Базовые параметры движения
                baseAngle: (Math.random() - 0.5) * 60, // начальный угол ±30°
                baseDistance: 2000 + Math.random() * 3000, // базовая дистанция 2-5м

                // Скорости движения (увеличены на 30%)
                angleSpeed: 0.4 + Math.random() * 0.2, // скорость изменения угла (0.4-0.6 рад/с)
                distanceSpeed: 0.25 + Math.random() * 0.15, // скорость изменения дистанции (0.25-0.4 м/с)

                // Фазы для плавности
                anglePhase: Math.random() * Math.PI * 2,
                distancePhase: Math.random() * Math.PI * 2,

                // Время смены направления
                directionChangeTime: currentTime + 5000 + Math.random() * 5000, // 5-10 сек

                // Скорость цели (м/с) - увеличено на 30%
                targetSpeed: 0.39 + Math.random() * 0.52, // 0.39-0.91 м/с

                // Время создания
                createdAt: target.createdAt || currentTime
            };
        }

        const params = target.movementParams;
        const lifeTime = (currentTime - params.createdAt) / 1000.0; // время жизни в секундах

        // Проверяем, нужно ли сменить направление
        if (currentTime > params.directionChangeTime) {
            params.baseAngle = (Math.random() - 0.5) * 60;
            params.baseDistance = 2000 + Math.random() * 3000;
            params.angleSpeed = 0.4 + Math.random() * 0.2;
            params.distanceSpeed = 0.25 + Math.random() * 0.15;
            params.anglePhase = Math.random() * Math.PI * 2;
            params.distancePhase = Math.random() * Math.PI * 2;
            params.directionChangeTime = currentTime + 5000 + Math.random() * 5000;
        }

        // Горизонтальное движение в пределах ±60°
        const hAngle = params.baseAngle + 30 * Math.sin(lifeTime * params.angleSpeed + params.anglePhase);

        // Дистанция с большим размахом - от 1м до 6.5м
        const distance = Math.max(1000, Math.min(6500, params.baseDistance + 2000 * Math.sin(lifeTime * params.distanceSpeed + params.distancePhase)));

        // Вычисляем координаты
        const angleRad = hAngle * Math.PI / 180;
        const x = Math.sin(angleRad) * distance;
        const y = Math.cos(angleRad) * distance;

        // Обновляем позицию цели
        target.x = x;
        target.y = y;
        target.distance = distance;
        target.angle = hAngle;

        // Реалистичная скорость человека (0.39-0.91 м/с) - увеличено на 30%
        target.speed = (params.targetSpeed * 1000); // конвертируем в мм/с

        // Обновляем время последнего обновления
        target.lastUpdate = currentTime;
    }

    // Имитация CAN сообщения
    createCANMessage(id, data) {
        return {
            id: id,
            data: data,
            timestamp: Date.now()
        };
    }

    // Генерация данных цели с синусоидальным движением
    generateTargetData(target) {
        // Обновляем позицию с размашистым движением
        this.updateTargetPosition(target);

        // Определяем состояние цели
        const state = this.checkTargetState(target);
        target.state = state;

        // Устанавливаем inSafetyZone для совместимости
        target.inSafetyZone = target.distance < this.options.safetyZone.minDistance;

        // Логика исчезновения целей
        const currentTime = Date.now();

        // Если цель вышла из FOV или за пределы диапазона, начинаем отсчет времени
        if (state === 'out_of_fov' || state === 'out_of_range') {
            if (!target.outOfFOVTime) {
                target.outOfFOVTime = currentTime;
            }
            // Исчезаем через 1 секунду после выхода из FOV
            if (currentTime - target.outOfFOVTime > 1000) {
                target.valid = false;
                target.outOfFOVTime = null;
            }
        } else {
            // Если цель вернулась в FOV, сбрасываем таймер
            target.outOfFOVTime = null;
        }

        // Убираем случайное исчезновение целей - они должны двигаться постоянно

        return target;
    }

    // Упаковка данных цели в CAN формат
    packTargetData(target) {
        const data = new Uint8Array(8);

        // X координата (16-bit, signed)
        const x = Math.max(-32768, Math.min(32767, Math.round(target.x)));
        data[0] = (x >> 8) & 0xFF;
        data[1] = x & 0xFF;

        // Y координата (16-bit, signed)
        const y = Math.max(-32768, Math.min(32767, Math.round(target.y)));
        data[2] = (y >> 8) & 0xFF;
        data[3] = y & 0xFF;

        // Расстояние (16-bit, unsigned)
        const distance = Math.max(0, Math.min(65535, Math.round(target.distance)));
        data[4] = (distance >> 8) & 0xFF;
        data[5] = distance & 0xFF;

        // Скорость (16-bit, signed)
        const speed = Math.max(-32768, Math.min(32767, Math.round(target.speed)));
        data[6] = (speed >> 8) & 0xFF;
        data[7] = speed & 0xFF;

        return Array.from(data);
    }

    // Проверка состояния цели
    checkTargetState(target) {
        // Проверяем FOV (±60° горизонтально, 6м дистанция для активных целей)
        const angleDeg = Math.atan2(target.y, target.x) * 180 / Math.PI;
        const inHorizontalFOV = Math.abs(angleDeg) <= 60;
        const inRange = target.distance <= 6000; // 6м для активных целей
        const inFOV = inHorizontalFOV && inRange;

        // Проверяем вылет за FOV (до 6.5м)
        const outOfRange = target.distance > 6500; // 6.5м - полный вылет

        // Проверяем safety zone (дистанция меньше 500mm)
        const inSafetyZone = target.distance < this.options.safetyZone.minDistance;

        // Определяем состояние
        if (inSafetyZone) {
            return 'safety'; // Красный цвет
        } else if (inFOV) {
            return 'active'; // Зеленый цвет
        } else if (outOfRange) {
            return 'out_of_range'; // Полный вылет за 6.5м
        } else {
            return 'out_of_fov'; // Синий цвет - в пределах 6-6.5м но вне угла FOV
        }
    }

    // Отправка сообщения
    sendMessage(msg) {
        this.emit('message', msg);
        this.stats.messagesSent++;
    }

    // Генерация и отправка данных
    generateData() {
        if (!this.isRunning) return;

        // Переинициализируем исчезнувшие цели
        for (let i = 0; i < this.targets.length; i++) {
            const target = this.targets[i];
            if (!target.valid) {
                // Переинициализируем исчезнувшую цель
                target.valid = true;
                target.createdAt = Date.now();
                target.lastUpdate = Date.now();
                target.outOfFOVTime = null;
                this.updateTargetPosition(target);
                console.log(`🔄 Reinitialized target ${i} at ${(target.distance/1000).toFixed(1)}m, ${target.angle.toFixed(1)}°`);
            }
        }

        // Обновляем цели
        for (let i = 0; i < this.targets.length; i++) {
            const target = this.targets[i];
            this.generateTargetData(target);

            if (target.valid) {
                // Отправляем данные цели
                const targetData = this.packTargetData(target);
                const targetId = CAN_IDS.TARGET_0 + i;
                const targetMsg = this.createCANMessage(targetId, targetData);
                this.sendMessage(targetMsg);

                // Проверяем состояние цели
                if (target.state === 'safety') {
                    console.log(`🚨 Target ${i} in safety zone: ${target.distance.toFixed(0)}mm`);
                } else if (target.state === 'out_of_fov') {
                    console.log(`🔵 Target ${i} out of FOV: ${target.distance.toFixed(0)}mm, angle: ${target.angle.toFixed(1)}°`);
                } else {
                    console.log(`🟢 Target ${i} in FOV: ${target.distance.toFixed(0)}mm, angle: ${target.angle.toFixed(1)}°`);
                }
            }
        }

        // Отправляем статус
        const statusData = [
            this.systemStatus,           // System status
            this.targets.filter(t => t.valid).length, // Active targets
            (this.options.safetyZone.minDistance >> 8) & 0xFF, // Safety distance high
            this.options.safetyZone.minDistance & 0xFF  // Safety distance low
        ];
        const statusMsg = this.createCANMessage(CAN_IDS.STATUS, statusData);
        this.sendMessage(statusMsg);

        // Убираем случайную аварийную остановку - она должна быть только при реальной опасности
    }

    // Запуск Mock CAN
    start() {
        if (this.isRunning) return;

        console.log('🚀 Starting Mock CAN Interface...');
        console.log(`📡 Channel: ${this.options.channel}`);
        console.log(`⚡ Bitrate: ${this.options.bitrate} bps`);
        console.log(`🎯 Targets: ${this.options.targetCount}`);
        console.log(`📊 Update interval: ${this.options.updateInterval}ms`);

        this.isRunning = true;

        // Запускаем генерацию данных
        this.intervalId = setInterval(() => {
            this.generateData();
        }, this.options.updateInterval);

        console.log('✅ Mock CAN Interface started!');

        // Статистика каждые 10 секунд
        this.statsIntervalId = setInterval(() => {
            this.printStatistics();
        }, 10000);
    }

    // Остановка Mock CAN
    stop() {
        if (!this.isRunning) return;

        console.log('🛑 Stopping Mock CAN Interface...');

        this.isRunning = false;

        if (this.intervalId) {
            clearInterval(this.intervalId);
        }

        if (this.statsIntervalId) {
            clearInterval(this.statsIntervalId);
        }

        console.log('✅ Mock CAN Interface stopped!');
    }

    // Печать статистики
    printStatistics() {
        const uptime = Math.floor((Date.now() - this.stats.startTime) / 1000);
        const activeTargets = this.targets.filter(t => t.valid).length;

        console.log(`📊 Mock CAN Statistics (uptime: ${uptime}s):`);
        console.log(`   📤 Messages sent: ${this.stats.messagesSent}`);
        console.log(`   🎯 Active targets: ${activeTargets}/${this.options.targetCount}`);
        console.log(`   🚨 Emergency stop: ${this.emergencyStop ? 'ACTIVE' : 'INACTIVE'}`);
        console.log(`   📡 System status: 0x${this.systemStatus.toString(16)}`);
    }

    // Получение текущих целей
    getTargets() {
        return this.targets.filter(t => t.valid);
    }

    // Установка зоны безопасности
    setSafetyZone(minDistance, maxDistance, angleRange) {
        this.options.safetyZone = {
            minDistance,
            maxDistance,
            angleRange
        };
        console.log(`⚙️ Safety zone updated: ${minDistance}-${maxDistance}mm, ±${angleRange/2}°`);
    }

    // Имитация подключения к CAN интерфейсу
    createRawChannel(channel, options) {
        console.log(`🔌 Mock CAN channel created: ${channel}`);
        console.log(`⚙️ Options:`, options);
        return this;
    }
}

// Экспорт для использования в других модулях
module.exports = MockCANInterface;

// Запуск если файл выполняется напрямую
if (require.main === module) {
    const mockCAN = new MockCANInterface();

    // Обработка сигналов для корректного завершения
    process.on('SIGINT', () => {
        console.log('\n🛑 Shutting down Mock CAN...');
        mockCAN.stop();
        process.exit(0);
    });

    process.on('SIGTERM', () => {
        console.log('\n🛑 Shutting down Mock CAN...');
        mockCAN.stop();
        process.exit(0);
    });

    // Запуск
    mockCAN.start();

    // Демонстрация работы
    console.log('🎯 Mock CAN Interface running...');
    console.log('📡 Simulating LD2450 radar data...');
    console.log('🔍 Press Ctrl+C to stop');
}
