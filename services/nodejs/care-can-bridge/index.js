#!/usr/bin/env node

/**
 * @file index.js
 * @module care-can-bridge
 * @description C.A.R.E. CAN Bridge Service - Мост между CAN шиной и ROS2
 * @author C.A.R.E. Development Team
 * @version 1.0.0
 *
 * @description
 * Этот сервис является связующим звеном между микроконтроллерами (ESP32/STM32)
 * и системой ROS2. Обрабатывает CAN сообщения и публикует данные в ROS2 топики.
 *
 * ## Основные функции:
 * - Приём CAN сообщений с радара LD2450 (через ESP32/STM32)
 * - Парсинг данных целей (ID 0x200-0x202)
 * - Обработка emergency stop (ID 0x100)
 * - Публикация данных в ROS2 топики
 * - Поддержка Mock режима для тестирования без железа
 *
 * ## CAN Message IDs:
 * - `0x100` - Emergency Stop (экстренная остановка)
 * - `0x200` - Target 0 data (цель #0: x, y)
 * - `0x201` - Target 1 data (цель #1: x, y)
 * - `0x202` - Target 2 data (цель #2: x, y)
 * - `0x300` - System Status (статус системы)
 * - `0x400` - Configuration (конфигурация)
 *
 * ## ROS2 Интерфейсы:
 * ### Publishers:
 * - `care_ld2450_driver/RadarTargets` - данные целей радара
 * - `care_ld2450_driver/SafetyStatus` - статус безопасности
 *
 * ### Services:
 * - `care_ld2450_driver/SetSafetyZone` - настройка зоны безопасности
 *
 * ## Режимы работы:
 * ### Real CAN Mode (реальное железо):
 * - Использует socketcan для работы с физическим CAN интерфейсом
 * - Требует rclnodejs для ROS2 интеграции
 * - Настраивается через CARE_SENSOR_TYPE=real
 *
 * ### Mock CAN Mode (симуляция):
 * - Генерирует синтетические CAN сообщения
 * - Не требует физического оборудования
 * - Настраивается через CARE_SENSOR_TYPE=mock
 *
 * @example
 * # Real mode
 * CARE_SENSOR_TYPE=real node index.js
 *
 * # Mock mode (тестирование)
 * CARE_SENSOR_TYPE=mock node index.js
 */

// Конфигурация датчика
const sensorConfig = require('../config/sensor-config.js');

// Условный импорт в зависимости от типа датчика
let CANInterface;
if (sensorConfig.isMockSensor()) {
    CANInterface = require('../mock-can/index.js');
    console.log('🎯 Using Mock CAN interface');
} else {
    const can = require('socketcan');
    CANInterface = can.createRawChannel;
    console.log('📡 Using real CAN interface');
}

// Условный импорт rclnodejs только для реального CAN
let rclnodejs = null;
if (!sensorConfig.isMockSensor()) {
    try {
        rclnodejs = require('rclnodejs');
    } catch (error) {
        console.error('❌ rclnodejs not available:', error.message);
        console.log('🔧 Install ROS 2 development packages:');
        console.log('   sudo apt-get install ros-jazzy-rclcpp-dev ros-jazzy-rcl-dev');
        process.exit(1);
    }
}
const fs = require('fs');
const path = require('path');

// CAN Message IDs for C.A.R.E.
const CAN_IDS = {
    EMERGENCY_STOP: 0x100,
    TARGET_0: 0x200,
    TARGET_1: 0x201,
    TARGET_2: 0x202,
    STATUS: 0x300,
    CONFIG: 0x400
};

/**
 * C.A.R.E. CAN Bridge - Мост между CAN и ROS2
 * @class
 */
class CareCANBridge {
    /**
     * Создаёт новый экземпляр CAN моста
     * @constructor
     */
    constructor() {
        /**
         * @private
         * @type {Object|null}
         * @description CAN канал (socketcan или mock)
         */
        this.canChannel = null;

        /**
         * @private
         * @type {Object|null}
         * @description ROS2 нода
         */
        this.rosNode = null;

        /**
         * @private
         * @type {Object|null}
         * @description Publisher для данных радара
         */
        this.radarPublisher = null;

        /**
         * @private
         * @type {Object|null}
         * @description Publisher для статуса безопасности
         */
        this.safetyPublisher = null;

        /**
         * @private
         * @type {Object|null}
         * @description ROS2 service для конфигурации
         */
        this.configService = null;

        /**
         * @private
         * @type {boolean}
         * @description Флаг состояния моста
         */
        this.isRunning = false;

        /**
         * @private
         * @type {Object}
         * @description Статистика обработки сообщений
         */
        this.stats = {
            messagesReceived: 0,
            messagesPublished: 0,
            errors: 0,
            startTime: Date.now()
        };
    }

    async initialize() {
        try {
            console.log('🚀 Initializing C.A.R.E. CAN Bridge...');

            // Initialize ROS 2 только для реального CAN
            if (rclnodejs) {
                await rclnodejs.init();
                this.rosNode = rclnodejs.createNode('care_can_bridge');
            } else {
                console.log('🎯 Mock CAN mode - skipping ROS 2 initialization');
                this.rosNode = null;
            }

            // Create publishers только для реального CAN
            if (this.rosNode) {
                this.radarPublisher = this.rosNode.createPublisher(
                    'care_ld2450_driver/RadarTargets',
                    'care_ld2450_driver/msg/RadarTargets'
                );

                this.safetyPublisher = this.rosNode.createPublisher(
                    'care_ld2450_driver/SafetyStatus',
                    'care_ld2450_driver/msg/SafetyStatus'
                );

                // Create service for configuration
                this.configService = this.rosNode.createService(
                    'care_ld2450_driver/SetSafetyZone',
                    'care_ld2450_driver/srv/SetSafetyZone',
                    this.handleConfigRequest.bind(this)
                );
            } else {
                console.log('🎯 Mock CAN mode - skipping ROS 2 publishers');
                this.radarPublisher = null;
                this.safetyPublisher = null;
                this.configService = null;
            }

            // Initialize CAN channel
            this.initializeCAN();

            console.log('✅ C.A.R.E. CAN Bridge initialized successfully');

        } catch (error) {
            console.error('❌ Failed to initialize CAN Bridge:', error);
            throw error;
        }
    }

    initializeCAN() {
        try {
            if (sensorConfig.isMockSensor()) {
                // Create Mock CAN channel for testing without hardware
                const mockConfig = sensorConfig.getSensorConfig();
                this.canChannel = new CANInterface({
                    channel: 'mock_can0',
                    bitrate: 500000,
                    ...mockConfig
                });

                // Handle CAN messages
                this.canChannel.on('message', this.handleCANMessage.bind(this));

                // Start Mock CAN channel
                this.canChannel.start();
                console.log('📡 Mock CAN channel started (500kbps)');
                console.log('🎯 Simulating LD2450 radar data...');

            } else {
                // Create real CAN channel
                const realConfig = sensorConfig.getSensorConfig();
                this.canChannel = CANInterface(realConfig.canInterface, {
                    bitrate: realConfig.bitrate
                });

                // Handle CAN messages
                this.canChannel.on('message', this.handleCANMessage.bind(this));
                this.canChannel.on('error', this.handleCANError.bind(this));

                // Start real CAN channel
                this.canChannel.start();
                console.log(`📡 Real CAN channel started on ${realConfig.canInterface} (${realConfig.bitrate}bps)`);
            }

        } catch (error) {
            console.error('❌ Failed to initialize CAN channel:', error);
            throw error;
        }
    }

    handleCANMessage(msg) {
        try {
            this.stats.messagesReceived++;

            const canId = msg.id;
            const data = msg.data;

            // Process different CAN message types
            switch (canId) {
                case CAN_IDS.EMERGENCY_STOP:
                    this.handleEmergencyStop(data);
                    break;

                case CAN_IDS.TARGET_0:
                case CAN_IDS.TARGET_1:
                case CAN_IDS.TARGET_2:
                    this.handleRadarTarget(canId, data);
                    break;

                case CAN_IDS.STATUS:
                    this.handleStatusMessage(data);
                    break;

                default:
                    console.log(`📨 Unknown CAN message: ID=0x${canId.toString(16)}, Data=[${data.join(', ')}]`);
            }

        } catch (error) {
            console.error('❌ Error handling CAN message:', error);
            this.stats.errors++;
        }
    }

    handleEmergencyStop(data) {
        const emergencyActive = data[0] === 0x01;

        const safetyMsg = {
            header: {
                stamp: { sec: 0, nanosec: 0 },
                frame_id: 'care_radar'
            },
            emergency_stop: emergencyActive,
            min_distance: 500.0,
            closest_target_id: -1
        };

        if (this.safetyPublisher) {
            this.safetyPublisher.publish(safetyMsg);
            this.stats.messagesPublished++;
        } else {
            console.log(`🚨 Emergency Stop: ${emergencyActive ? 'ACTIVE' : 'INACTIVE'}`);
        }

        console.log(`🚨 Emergency Stop: ${emergencyActive ? 'ACTIVE' : 'INACTIVE'}`);
    }

    handleRadarTarget(canId, data) {
        if (data.length < 8) return;

        const targetId = canId - CAN_IDS.TARGET_0;

        // Unpack target data (same format as in C++ code)
        const x = (data[0] << 8) | data[1];
        const y = (data[2] << 8) | data[3];
        const distance = (data[4] << 8) | data[5];
        const speed = (data[6] << 8) | data[7];

        const radarMsg = {
            header: {
                stamp: { sec: 0, nanosec: 0 },
                frame_id: 'care_radar'
            },
            targets: [{
                id: targetId,
                x: x,
                y: y,
                speed: speed,
                distance: distance,
                valid: true
            }]
        };

        if (this.radarPublisher) {
            this.radarPublisher.publish(radarMsg);
            this.stats.messagesPublished++;
        }

        console.log(`🎯 Target ${targetId}: X=${x}mm, Y=${y}mm, D=${distance}mm, S=${speed}mm/s`);
    }

    handleStatusMessage(data) {
        if (data.length < 4) return;

        const systemStatus = data[0];
        const activeTargets = data[1];
        const safetyDistance = (data[2] << 8) | data[3];

        console.log(`📊 Status: System=${systemStatus}, Targets=${activeTargets}, Safety=${safetyDistance}mm`);
    }

    handleConfigRequest(request, response) {
        try {
            console.log(`⚙️ Configuration request: min_distance=${request.min_safe_distance}mm, max_distance=${request.max_safe_distance}mm`);

            // Here you would send configuration to the microcontroller via CAN
            // For now, just acknowledge the request

            response.success = true;
            response.message = 'Configuration updated successfully';

            return response;

        } catch (error) {
            console.error('❌ Configuration error:', error);
            response.success = false;
            response.message = `Configuration failed: ${error.message}`;
            return response;
        }
    }

    handleCANError(error) {
        console.error('❌ CAN Error:', error);
        this.stats.errors++;
    }

    async start() {
        try {
            await this.initialize();
            this.isRunning = true;

            console.log('🎉 C.A.R.E. CAN Bridge started successfully!');
            console.log('📡 Listening for CAN messages...');
            console.log('🤖 ROS 2 node: care_can_bridge');

            // Print statistics every 10 seconds
            setInterval(() => {
                this.printStatistics();
            }, 10000);

        } catch (error) {
            console.error('❌ Failed to start CAN Bridge:', error);
            process.exit(1);
        }
    }

    printStatistics() {
        const uptime = Math.floor((Date.now() - this.stats.startTime) / 1000);
        console.log(`📊 Statistics (uptime: ${uptime}s):`);
        console.log(`   📨 CAN messages received: ${this.stats.messagesReceived}`);
        console.log(`   📤 ROS messages published: ${this.stats.messagesPublished}`);
        console.log(`   ❌ Errors: ${this.stats.errors}`);
    }

    async stop() {
        try {
            if (this.canChannel) {
                this.canChannel.stop();
            }

            if (this.rosNode) {
                this.rosNode.destroy();
            }

            this.isRunning = false;
            console.log('🛑 C.A.R.E. CAN Bridge stopped');

        } catch (error) {
            console.error('❌ Error stopping CAN Bridge:', error);
        }
    }
}

// Main execution
async function main() {
    const bridge = new CareCANBridge();

    // Handle graceful shutdown
    process.on('SIGINT', async () => {
        console.log('\n🛑 Shutting down C.A.R.E. CAN Bridge...');
        await bridge.stop();
        process.exit(0);
    });

    process.on('SIGTERM', async () => {
        console.log('\n🛑 Shutting down C.A.R.E. CAN Bridge...');
        await bridge.stop();
        process.exit(0);
    });

    await bridge.start();
}

// Run if this file is executed directly
if (require.main === module) {
    main().catch(error => {
        console.error('❌ Fatal error:', error);
        process.exit(1);
    });
}

module.exports = CareCANBridge;
