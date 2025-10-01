#!/usr/bin/env node

/**
 * @file index.js
 * @module care-dashboard
 * @description C.A.R.E. Web Dashboard - Веб-интерфейс реального времени для мониторинга
 * @author C.A.R.E. Development Team
 * @version 1.0.0
 *
 * @description
 * Веб-дашборд предоставляет интерфейс для мониторинга и визуализации данных
 * радарной системы C.A.R.E. в реальном времени через WebSocket.
 *
 * ## Основные функции:
 * - Real-time визуализация целей радара
 * - Отображение статуса безопасности (emergency stop)
 * - Статистика системы (uptime, messages/sec, latency)
 * - WebSocket для live обновлений данных
 * - REST API для получения снимков данных
 *
 * ## Endpoints:
 * - `GET /` - главная страница дашборда
 * - `GET /api/status` - полный статус системы
 * - `GET /api/radar` - данные радара
 * - `GET /api/safety` - статус безопасности
 * - `GET /api/statistics` - статистика
 * - `WS ws://localhost:3000` - WebSocket для live данных
 *
 * ## WebSocket Protocol:
 * Отправляет JSON сообщения каждые 100мс:
 * ```json
 * {
 *   "type": "update",
 *   "data": {
 *     "radar": { "targets": [...], "activeTargets": 3 },
 *     "safety": { "emergencyStop": false, "minDistance": 500 },
 *     "system": { "status": "ok", "uptime": 12345 }
 *   },
 *   "timestamp": "2025-10-01T12:00:00.000Z"
 * }
 * ```
 *
 * @example
 * # Запуск на порту 3000
 * node index.js
 *
 * # Кастомный порт
 * PORT=8080 node index.js
 *
 * # Откройте в браузере
 * http://localhost:3000
 */

const express = require('express');
const WebSocket = require('ws');
const http = require('http');
const path = require('path');
const cors = require('cors');
const helmet = require('helmet');
const morgan = require('morgan');
const moment = require('moment');

class CareDashboard {
    constructor(port = 3000) {
        this.port = port;
        this.app = express();
        this.server = http.createServer(this.app);
        this.wss = new WebSocket.Server({ server: this.server });
        this.clients = new Set();

        // Dashboard data
        this.dashboardData = {
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
                status: 'unknown',
                uptime: 0,
                canMessages: 0,
                rosMessages: 0,
                errors: 0
            },
            statistics: {
                messagesPerSecond: 0,
                averageLatency: 0,
                peakLatency: 0
            }
        };

        this.setupMiddleware();
        this.setupRoutes();
        this.setupWebSocket();
    }

    setupMiddleware() {
        // Security
        this.app.use(helmet({
            contentSecurityPolicy: {
                directives: {
                    defaultSrc: ["'self'"],
                    styleSrc: ["'self'", "'unsafe-inline'", "https://cdnjs.cloudflare.com"],
                    scriptSrc: ["'self'", "'unsafe-inline'", "https://cdnjs.cloudflare.com"],
                    connectSrc: ["'self'", "ws://localhost:3000"]
                }
            }
        }));

        // CORS
        this.app.use(cors());

        // Logging
        this.app.use(morgan('combined'));

        // Static files
        this.app.use(express.static(path.join(__dirname, 'public')));
        this.app.use(express.json());
    }

    setupRoutes() {
        // Main dashboard
        this.app.get('/', (req, res) => {
            res.sendFile(path.join(__dirname, 'public', 'index.html'));
        });

        // API endpoints
        this.app.get('/api/status', (req, res) => {
            res.json({
                success: true,
                data: this.dashboardData,
                timestamp: moment().toISOString()
            });
        });

        this.app.get('/api/radar', (req, res) => {
            res.json({
                success: true,
                data: this.dashboardData.radar,
                timestamp: moment().toISOString()
            });
        });

        this.app.get('/api/safety', (req, res) => {
            res.json({
                success: true,
                data: this.dashboardData.safety,
                timestamp: moment().toISOString()
            });
        });

        this.app.get('/api/statistics', (req, res) => {
            res.json({
                success: true,
                data: this.dashboardData.statistics,
                timestamp: moment().toISOString()
            });
        });

        // Health check
        this.app.get('/health', (req, res) => {
            res.json({
                status: 'healthy',
                timestamp: moment().toISOString(),
                uptime: process.uptime()
            });
        });
    }

    setupWebSocket() {
        this.wss.on('connection', (ws, req) => {
            console.log('🔌 New WebSocket client connected');
            this.clients.add(ws);

            // Send initial data
            ws.send(JSON.stringify({
                type: 'initial_data',
                data: this.dashboardData
            }));

            ws.on('close', () => {
                console.log('🔌 WebSocket client disconnected');
                this.clients.delete(ws);
            });

            ws.on('error', (error) => {
                console.error('❌ WebSocket error:', error);
                this.clients.delete(ws);
            });
        });
    }

    updateRadarData(targets) {
        this.dashboardData.radar.targets = targets;
        this.dashboardData.radar.lastUpdate = moment().toISOString();
        this.dashboardData.radar.activeTargets = targets.filter(t => t.valid).length;

        this.broadcastUpdate('radar', this.dashboardData.radar);
    }

    updateSafetyStatus(safety) {
        this.dashboardData.safety = {
            ...this.dashboardData.safety,
            ...safety,
            lastUpdate: moment().toISOString()
        };

        this.broadcastUpdate('safety', this.dashboardData.safety);
    }

    updateSystemStatus(system) {
        this.dashboardData.system = {
            ...this.dashboardData.system,
            ...system,
            lastUpdate: moment().toISOString()
        };

        this.broadcastUpdate('system', this.dashboardData.system);
    }

    updateStatistics(stats) {
        this.dashboardData.statistics = {
            ...this.dashboardData.statistics,
            ...stats,
            lastUpdate: moment().toISOString()
        };

        this.broadcastUpdate('statistics', this.dashboardData.statistics);
    }

    broadcastUpdate(type, data) {
        const message = JSON.stringify({
            type: type,
            data: data,
            timestamp: moment().toISOString()
        });

        this.clients.forEach(client => {
            if (client.readyState === WebSocket.OPEN) {
                client.send(message);
            }
        });
    }

    start() {
        this.server.listen(this.port, () => {
            console.log(`🌐 C.A.R.E. Dashboard running on http://localhost:${this.port}`);
            console.log(`📡 WebSocket server ready for real-time updates`);
        });
    }

    stop() {
        this.server.close(() => {
            console.log('🛑 C.A.R.E. Dashboard stopped');
        });
    }
}

// Main execution
function main() {
    const dashboard = new CareDashboard(process.env.PORT || 3000);

    // Handle graceful shutdown
    process.on('SIGINT', () => {
        console.log('\n🛑 Shutting down C.A.R.E. Dashboard...');
        dashboard.stop();
        process.exit(0);
    });

    process.on('SIGTERM', () => {
        console.log('\n🛑 Shutting down C.A.R.E. Dashboard...');
        dashboard.stop();
        process.exit(0);
    });

    dashboard.start();
}

// Run if this file is executed directly
if (require.main === module) {
    main();
}

module.exports = CareDashboard;
