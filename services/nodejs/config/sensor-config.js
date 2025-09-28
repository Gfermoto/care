/**
 * C.A.R.E. Sensor Configuration
 * 
 * Конфигурация для выбора между реальным датчиком и Mock CAN
 */

module.exports = {
    // Тип датчика: 'real' или 'mock'
    sensorType: process.env.CARE_SENSOR_TYPE || 'mock',
    
    // Конфигурация для реального датчика
    realSensor: {
        canInterface: 'can0',
        bitrate: 500000,
        timeout: 1000,
        retries: 3
    },
    
    // Конфигурация для Mock датчика
    mockSensor: {
        targetCount: 3,
        updateInterval: 100,
        safetyZone: {
            minDistance: 500,    // мм
            maxDistance: 8000,   // мм
            angleRange: 120      // градусы
        },
        simulation: {
            enableMovement: true,
            enableNoise: true,
            enableTargets: true
        }
    },
    
    // Общие настройки
    common: {
        canIds: {
            EMERGENCY_STOP: 0x100,
            TARGET_0: 0x200,
            TARGET_1: 0x201,
            TARGET_2: 0x202,
            STATUS: 0x300,
            CONFIG: 0x400
        },
        logLevel: 'info'
    },
    
    // Получить конфигурацию для текущего типа датчика
    getSensorConfig() {
        return this.sensorType === 'real' ? this.realSensor : this.mockSensor;
    },
    
    // Проверить, используется ли Mock датчик
    isMockSensor() {
        return this.sensorType === 'mock';
    },
    
    // Проверить, используется ли реальный датчик
    isRealSensor() {
        return this.sensorType === 'real';
    },
    
    // Получить информацию о конфигурации
    getConfigInfo() {
        return {
            sensorType: this.sensorType,
            config: this.getSensorConfig(),
            timestamp: new Date().toISOString()
        };
    }
};
