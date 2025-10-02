#!/usr/bin/env node

/**
 * C.A.R.E. Integrated Node.js Demo
 * Объединяет Mock CAN, Web Dashboard и API Server в одном процессе
 */

const MockCANInterface = require('./mock-can/index.js');
const CareDashboard = require('./care-dashboard/index.js');
const CareAPI = require('./care-api/index.js');

class CareIntegratedDemo {
    constructor() {
        this.mockCAN = null;
        this.dashboard = null;
        this.api = null;
        this.isRunning = false;

        // Данные для передачи между компонентами
        this.sharedData = {
            radar: {
                targets: [],
                lastUpdate: null,
                activeTargets: 0
            },
            safety: {
                emergencyStop: false,
                minDistance: 500,
                activeTargets: 0,
                lastTrigger: null
            },
            system: {
                status: 'ok',
                uptime: 0,
                canMessages: 0,
                rosMessages: 0,
                errors: 0
            }
        };
    }

    async start() {
        try {
            console.log('🚀 Starting C.A.R.E. Integrated Node.js Demo...');

            // Start API Server first
            console.log('📊 Starting API Server...');
            this.api = new CareAPI(3001);
            this.api.start();

            // Start Web Dashboard
            console.log('🌐 Starting Web Dashboard...');
            this.dashboard = new CareDashboard(3000);
            this.dashboard.start();

            // Start Mock CAN Interface
            console.log('🎯 Starting Mock CAN Interface...');
            this.mockCAN = new MockCANInterface({
                targetCount: 3,
                updateInterval: 100, // ms
                safetyZone: {
                    minDistance: 500,
                    maxDistance: 8000,
                    angleRange: 120
                }
            });

            // Connect Mock CAN to Dashboard
            this.mockCAN.on('message', (msg) => {
                this.handleCANMessage(msg);
            });

            this.mockCAN.start();

            // Start data synchronization
            this.startDataSync();

            this.isRunning = true;

            console.log('🎉 C.A.R.E. Integrated Demo started successfully!');
            console.log('📊 Web Dashboard: http://localhost:3000');
            console.log('🔌 API Server: http://localhost:3001');
            console.log('🎯 Mock CAN: Generating radar data...');

        } catch (error) {
            console.error('❌ Failed to start C.A.R.E. Demo:', error);
            await this.stop();
            process.exit(1);
        }
    }

    handleCANMessage(msg) {
        try {
            const canId = msg.id;
            const data = msg.data;

            // Update system statistics
            this.sharedData.system.canMessages++;
            this.sharedData.system.uptime = Math.floor((Date.now() - this.startTime) / 1000);

            switch (canId) {
                case 0x100: // Emergency Stop
                    this.handleEmergencyStop(data);
                    break;

                case 0x200: // Target 0
                case 0x201: // Target 1
                case 0x202: // Target 2
                    this.handleRadarTarget(canId, data);
                    break;

                case 0x300: // System Status
                    this.handleStatusMessage(data);
                    break;
            }

        } catch (error) {
            console.error('❌ Error handling CAN message:', error);
            this.sharedData.system.errors++;
        }
    }

    handleEmergencyStop(data) {
        const emergencyActive = data[0] === 0x01;

        this.sharedData.safety.emergencyStop = emergencyActive;
        this.sharedData.safety.lastTrigger = emergencyActive ? new Date().toISOString() : null;

        // Update dashboard
        this.dashboard.updateSafetyStatus(this.sharedData.safety);

        console.log(`🚨 Emergency Stop: ${emergencyActive ? 'ACTIVE' : 'INACTIVE'}`);
    }

    handleRadarTarget(canId, data) {
        if (data.length < 8) return;

        const targetId = canId - 0x200;

        // Unpack target data
        const x = (data[0] << 8) | data[1];
        const y = (data[2] << 8) | data[3];
        const distance = (data[4] << 8) | data[5];
        const speed = (data[6] << 8) | data[7];

        // Update target in shared data
        if (!this.sharedData.radar.targets[targetId]) {
            this.sharedData.radar.targets[targetId] = {};
        }

        this.sharedData.radar.targets[targetId] = {
            id: targetId,
            x: x,
            y: y,
            distance: distance,
            speed: speed,
            valid: true,
            lastUpdate: Date.now()
        };

        this.sharedData.radar.lastUpdate = new Date().toISOString();
        this.sharedData.radar.activeTargets = this.sharedData.radar.targets.filter(t => t && t.valid).length;

        // Update dashboard
        this.dashboard.updateRadarDisplay(this.sharedData.radar);

        // Update safety status
        this.sharedData.safety.activeTargets = this.sharedData.radar.activeTargets;
        this.dashboard.updateSafetyStatus(this.sharedData.safety);

        console.log(`🎯 Target ${targetId}: X=${x}mm, Y=${y}mm, D=${distance}mm, S=${speed}mm/s`);
    }

    handleStatusMessage(data) {
        if (data.length < 4) return;

        const systemStatus = data[0];
        const activeTargets = data[1];
        const safetyDistance = (data[2] << 8) | data[3];

        this.sharedData.system.status = systemStatus === 0x01 ? 'ok' : 'error';
        this.sharedData.safety.minDistance = safetyDistance;

        // Update dashboard
        this.dashboard.updateSystemStatus(this.sharedData.system);

        console.log(`📊 Status: System=${systemStatus}, Targets=${activeTargets}, Safety=${safetyDistance}mm`);
    }

    startDataSync() {
        // Sync data every 100ms
        setInterval(() => {
            if (this.isRunning) {
                // Update statistics
                const now = Date.now();
                this.sharedData.system.uptime = Math.floor((now - this.startTime) / 1000);

                // Update dashboard with current statistics
                const stats = {
                    messagesPerSecond: this.sharedData.system.canMessages / Math.max(1, this.sharedData.system.uptime),
                    averageLatency: 50, // Mock latency
                    peakLatency: 100
                };

                this.dashboard.updateStatistics(stats);
            }
        }, 100);
    }

    async stop() {
        try {
            console.log('🛑 Stopping C.A.R.E. Integrated Demo...');

            if (this.mockCAN) {
                this.mockCAN.stop();
            }

            if (this.dashboard) {
                this.dashboard.stop();
            }

            if (this.api) {
                this.api.stop();
            }

            this.isRunning = false;
            console.log('✅ C.A.R.E. Integrated Demo stopped');

        } catch (error) {
            console.error('❌ Error stopping demo:', error);
        }
    }

    getStatus() {
        return {
            running: this.isRunning,
            data: this.sharedData,
            timestamp: new Date().toISOString()
        };
    }
}

// Main execution
async function main() {
    const demo = new CareIntegratedDemo();
    demo.startTime = Date.now();

    // Handle graceful shutdown
    process.on('SIGINT', async () => {
        console.log('\n🛑 Shutting down C.A.R.E. Integrated Demo...');
        await demo.stop();
        process.exit(0);
    });

    process.on('SIGTERM', async () => {
        console.log('\n🛑 Shutting down C.A.R.E. Integrated Demo...');
        await demo.stop();
        process.exit(0);
    });

    await demo.start();
}

// Run if this file is executed directly
if (require.main === module) {
    main().catch(error => {
        console.error('❌ Fatal error:', error);
        process.exit(1);
    });
}

module.exports = CareIntegratedDemo;
