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

        this.initializeTargets();
    }

    initializeTargets() {
        // КРИТИЧНО: Очищаем массив целей перед инициализацией!
        this.targets = [];

        // Инициализация случайных целей СТРОГО в пределах FOV и 6.5м
        for (let i = 0; i < this.options.targetCount; i++) {
            // FOV от -60° до +60° (это 120° сектор от 300° до 60° через 0°)
            const angle = Math.random() * 120 - 60; // -60° to +60°
            const distance = Math.random() * 5500 + 1000; // 1-6.5м (в миллиметрах)

            // ВАЖНО: Вычисляем координаты так, чтобы distance = sqrt(x^2 + y^2)
            const angleRad = angle * Math.PI / 180;
            const x = Math.sin(angleRad) * distance; // X координата в мм
            const y = Math.cos(angleRad) * distance; // Y координата в мм

            // ПРОВЕРКА: расстояние должно быть точно distance
            const actualDistance = Math.sqrt(x * x + y * y);

            console.log(`🎯 Init Target ${i}: angle=${angle.toFixed(1)}°, distance=${distance.toFixed(0)}mm, actual=${actualDistance.toFixed(0)}mm, x=${x.toFixed(0)}mm, y=${y.toFixed(0)}mm`);

            if (actualDistance > 6500) {
                console.error(`❌ ERROR: Target ${i} exceeded 6.5m: ${actualDistance.toFixed(0)}mm!`);
            }

            this.targets.push({
                id: i,
                x: x,
                y: y,
                distance: actualDistance, // Используем реальное расстояние
                speed: (Math.random() - 0.5) * 100,
                angle: angle,
                valid: true,
                lastUpdate: Date.now()
            });
        }

        console.log(`✅ Initialized ${this.targets.length} targets within 6.5m`);
    }

    // Имитация CAN сообщения
    createCANMessage(id, data) {
        return {
            id: id,
            data: data,
            timestamp: Date.now()
        };
    }

    // Генерация данных цели
    generateTargetData(target) {
        // Более медленное движение целей для стабильности
        const moveSpeed = 20; // мм за обновление (уменьшил с 50 до 20)
        target.x += (Math.random() - 0.5) * moveSpeed;
        target.y += (Math.random() - 0.5) * moveSpeed;

        // СНАЧАЛА проверяем и ограничиваем расстояние ДО вычисления угла!
        target.distance = Math.sqrt(target.x * target.x + target.y * target.y);

        // СТРОГО держим цели в пределах 6.5м
        if (target.distance > 6500) {
            console.log(`🚨 Target ${target.id} exceeded 6.5m: ${target.distance.toFixed(0)}mm, adjusting...`);
            console.log(`   Before: x=${target.x.toFixed(0)}mm, y=${target.y.toFixed(0)}mm`);
            // Отскакиваем от границы
            const factor = 6500 / target.distance;
            target.x *= factor;
            target.y *= factor;
            target.distance = 6500;
            console.log(`   After: x=${target.x.toFixed(0)}mm, y=${target.y.toFixed(0)}mm, distance=${target.distance}mm`);
        }

        // ТЕПЕРЬ вычисляем угол с правильными координатами
        target.angle = Math.atan2(target.x, target.y) * 180 / Math.PI;

        // Проверяем FOV (300°-60° через 0°)
        let azimuthNorm = ((target.angle % 360) + 360) % 360;
        if (azimuthNorm >= 300 || azimuthNorm <= 60) {
            // Цель в FOV - все ОК
        } else {
            // Цель вне FOV - возвращаем в FOV
            if (azimuthNorm > 60 && azimuthNorm < 300) {
                // Между 60° и 300° - вне FOV
                if (azimuthNorm < 180) {
                    target.angle = 60; // ближе к 60°
                } else {
                    target.angle = 300; // ближе к 300°
                }
                // Пересчитываем координаты с ИСПРАВЛЕННЫМ расстоянием
                const angleRad = target.angle * Math.PI / 180;
                target.x = Math.sin(angleRad) * target.distance;
                target.y = Math.cos(angleRad) * target.distance;
            }
        }

        // Обновляем скорость (медленная)
        target.speed = (Math.random() - 0.5) * 100; // ±50mm/s

        // Иногда цель исчезает/появляется (реже)
        if (Math.random() < 0.01) {
            target.valid = !target.valid;
        }

        target.lastUpdate = Date.now();

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

    // Проверка зоны безопасности
    checkSafetyZone(target) {
        const inRange = target.distance >= this.options.safetyZone.minDistance &&
                       target.distance <= this.options.safetyZone.maxDistance;

        const inAngle = Math.abs(Math.atan2(target.y, target.x) * 180 / Math.PI) <=
                       this.options.safetyZone.angleRange / 2;

        return inRange && inAngle;
    }

    // Отправка сообщения
    sendMessage(msg) {
        this.emit('message', msg);
        this.stats.messagesSent++;
    }

    // Генерация и отправка данных
    generateData() {
        if (!this.isRunning) return;

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

                // Проверяем зону безопасности
                if (this.checkSafetyZone(target)) {
                    console.log(`🚨 Target ${i} in safety zone: ${target.distance.toFixed(0)}mm`);
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

        // Случайная аварийная остановка
        if (Math.random() < 0.001) { // 0.1% chance
            this.emergencyStop = !this.emergencyStop;
            const emergencyData = [this.emergencyStop ? 0x01 : 0x00];
            const emergencyMsg = this.createCANMessage(CAN_IDS.EMERGENCY_STOP, emergencyData);
            this.sendMessage(emergencyMsg);
            console.log(`🚨 Emergency Stop: ${this.emergencyStop ? 'ACTIVE' : 'INACTIVE'}`);
        }
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
