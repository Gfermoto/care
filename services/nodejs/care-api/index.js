#!/usr/bin/env node

/**
 * C.A.R.E. API Service
 * 
 * RESTful API for C.A.R.E. radar system configuration and data access
 * Provides endpoints for radar configuration, safety settings, and data logging
 */

const express = require('express');
const cors = require('cors');
const helmet = require('helmet');
const morgan = require('morgan');
const moment = require('moment');
const createCsvWriter = require('csv-writer').createObjectCsvWriter;
const fs = require('fs');
const path = require('path');

class CareAPI {
    constructor(port = 3001) {
        this.port = port;
        this.app = express();
        
        // Data storage
        this.radarData = [];
        this.safetyLogs = [];
        this.systemLogs = [];
        
        // CSV writers
        this.setupCSVWriters();
        
        this.setupMiddleware();
        this.setupRoutes();
    }

    setupCSVWriters() {
        const dataDir = path.join(__dirname, 'data');
        if (!fs.existsSync(dataDir)) {
            fs.mkdirSync(dataDir, { recursive: true });
        }

        this.radarCsvWriter = createCsvWriter({
            path: path.join(dataDir, `radar_data_${moment().format('YYYY-MM-DD')}.csv`),
            header: [
                { id: 'timestamp', title: 'Timestamp' },
                { id: 'target_id', title: 'Target ID' },
                { id: 'x', title: 'X (mm)' },
                { id: 'y', title: 'Y (mm)' },
                { id: 'distance', title: 'Distance (mm)' },
                { id: 'speed', title: 'Speed (mm/s)' },
                { id: 'valid', title: 'Valid' }
            ]
        });

        this.safetyCsvWriter = createCsvWriter({
            path: path.join(dataDir, `safety_logs_${moment().format('YYYY-MM-DD')}.csv`),
            header: [
                { id: 'timestamp', title: 'Timestamp' },
                { id: 'emergency_stop', title: 'Emergency Stop' },
                { id: 'min_distance', title: 'Min Distance (mm)' },
                { id: 'active_targets', title: 'Active Targets' },
                { id: 'trigger_reason', title: 'Trigger Reason' }
            ]
        });
    }

    setupMiddleware() {
        // Security
        this.app.use(helmet());
        
        // CORS
        this.app.use(cors());
        
        // Logging
        this.app.use(morgan('combined'));
        
        // JSON parsing
        this.app.use(express.json({ limit: '10mb' }));
        this.app.use(express.urlencoded({ extended: true }));
    }

    setupRoutes() {
        // Health check
        this.app.get('/health', (req, res) => {
            res.json({
                status: 'healthy',
                timestamp: moment().toISOString(),
                uptime: process.uptime(),
                version: '1.0.0'
            });
        });

        // Radar data endpoints
        this.app.get('/api/radar/data', (req, res) => {
            const limit = parseInt(req.query.limit) || 100;
            const offset = parseInt(req.query.offset) || 0;
            
            const data = this.radarData
                .slice(offset, offset + limit)
                .map(entry => ({
                    ...entry,
                    timestamp: moment(entry.timestamp).toISOString()
                }));
            
            res.json({
                success: true,
                data: data,
                total: this.radarData.length,
                limit: limit,
                offset: offset
            });
        });

        this.app.post('/api/radar/data', (req, res) => {
            try {
                const radarData = {
                    timestamp: moment().toISOString(),
                    ...req.body
                };
                
                this.radarData.push(radarData);
                
                // Keep only last 1000 entries in memory
                if (this.radarData.length > 1000) {
                    this.radarData = this.radarData.slice(-1000);
                }
                
                // Log to CSV
                this.radarCsvWriter.writeRecords([radarData]).catch(console.error);
                
                res.json({
                    success: true,
                    message: 'Radar data recorded successfully'
                });
                
            } catch (error) {
                res.status(500).json({
                    success: false,
                    message: 'Failed to record radar data',
                    error: error.message
                });
            }
        });

        // Safety endpoints
        this.app.get('/api/safety/logs', (req, res) => {
            const limit = parseInt(req.query.limit) || 100;
            const offset = parseInt(req.query.offset) || 0;
            
            const data = this.safetyLogs
                .slice(offset, offset + limit)
                .map(entry => ({
                    ...entry,
                    timestamp: moment(entry.timestamp).toISOString()
                }));
            
            res.json({
                success: true,
                data: data,
                total: this.safetyLogs.length,
                limit: limit,
                offset: offset
            });
        });

        this.app.post('/api/safety/logs', (req, res) => {
            try {
                const safetyLog = {
                    timestamp: moment().toISOString(),
                    ...req.body
                };
                
                this.safetyLogs.push(safetyLog);
                
                // Keep only last 500 entries in memory
                if (this.safetyLogs.length > 500) {
                    this.safetyLogs = this.safetyLogs.slice(-500);
                }
                
                // Log to CSV
                this.safetyCsvWriter.writeRecords([safetyLog]).catch(console.error);
                
                res.json({
                    success: true,
                    message: 'Safety log recorded successfully'
                });
                
            } catch (error) {
                res.status(500).json({
                    success: false,
                    message: 'Failed to record safety log',
                    error: error.message
                });
            }
        });

        // Configuration endpoints
        this.app.get('/api/config/safety', (req, res) => {
            res.json({
                success: true,
                data: {
                    min_safe_distance: 500,
                    max_safe_distance: 5000,
                    safety_angle: 30,
                    emergency_stop_enabled: true,
                    auto_reset_delay: 5000
                }
            });
        });

        this.app.post('/api/config/safety', (req, res) => {
            try {
                const config = req.body;
                
                // Validate configuration
                if (config.min_safe_distance < 100 || config.min_safe_distance > 2000) {
                    return res.status(400).json({
                        success: false,
                        message: 'Invalid min_safe_distance (must be 100-2000mm)'
                    });
                }
                
                if (config.max_safe_distance < 1000 || config.max_safe_distance > 10000) {
                    return res.status(400).json({
                        success: false,
                        message: 'Invalid max_safe_distance (must be 1000-10000mm)'
                    });
                }
                
                // Here you would send configuration to the microcontroller
                console.log('⚙️ Safety configuration updated:', config);
                
                res.json({
                    success: true,
                    message: 'Safety configuration updated successfully',
                    data: config
                });
                
            } catch (error) {
                res.status(500).json({
                    success: false,
                    message: 'Failed to update safety configuration',
                    error: error.message
                });
            }
        });

        // Statistics endpoints
        this.app.get('/api/statistics', (req, res) => {
            const now = moment();
            const today = now.startOf('day');
            const yesterday = now.clone().subtract(1, 'day').startOf('day');
            
            const todayRadarData = this.radarData.filter(entry => 
                moment(entry.timestamp).isAfter(today)
            );
            
            const todaySafetyLogs = this.safetyLogs.filter(entry => 
                moment(entry.timestamp).isAfter(today)
            );
            
            const emergencyStops = todaySafetyLogs.filter(entry => 
                entry.emergency_stop === true
            ).length;
            
            res.json({
                success: true,
                data: {
                    radar: {
                        total_entries: this.radarData.length,
                        today_entries: todayRadarData.length,
                        unique_targets: new Set(todayRadarData.map(entry => entry.target_id)).size
                    },
                    safety: {
                        total_logs: this.safetyLogs.length,
                        today_logs: todaySafetyLogs.length,
                        emergency_stops_today: emergencyStops
                    },
                    system: {
                        uptime: process.uptime(),
                        memory_usage: process.memoryUsage(),
                        timestamp: moment().toISOString()
                    }
                }
            });
        });

        // Data export endpoints
        this.app.get('/api/export/radar', (req, res) => {
            const format = req.query.format || 'json';
            const date = req.query.date || moment().format('YYYY-MM-DD');
            
            if (format === 'csv') {
                const csvPath = path.join(__dirname, 'data', `radar_data_${date}.csv`);
                if (fs.existsSync(csvPath)) {
                    res.download(csvPath);
                } else {
                    res.status(404).json({
                        success: false,
                        message: 'CSV file not found for the specified date'
                    });
                }
            } else {
                const data = this.radarData.filter(entry => 
                    moment(entry.timestamp).format('YYYY-MM-DD') === date
                );
                
                res.json({
                    success: true,
                    data: data,
                    count: data.length,
                    date: date
                });
            }
        });

        this.app.get('/api/export/safety', (req, res) => {
            const format = req.query.format || 'json';
            const date = req.query.date || moment().format('YYYY-MM-DD');
            
            if (format === 'csv') {
                const csvPath = path.join(__dirname, 'data', `safety_logs_${date}.csv`);
                if (fs.existsSync(csvPath)) {
                    res.download(csvPath);
                } else {
                    res.status(404).json({
                        success: false,
                        message: 'CSV file not found for the specified date'
                    });
                }
            } else {
                const data = this.safetyLogs.filter(entry => 
                    moment(entry.timestamp).format('YYYY-MM-DD') === date
                );
                
                res.json({
                    success: true,
                    data: data,
                    count: data.length,
                    date: date
                });
            }
        });

        // Error handling
        this.app.use((error, req, res, next) => {
            console.error('❌ API Error:', error);
            res.status(500).json({
                success: false,
                message: 'Internal server error',
                error: error.message
            });
        });

        // 404 handler
        this.app.use((req, res) => {
            res.status(404).json({
                success: false,
                message: 'Endpoint not found',
                path: req.path
            });
        });
    }

    start() {
        this.app.listen(this.port, () => {
            console.log(`🚀 C.A.R.E. API Server running on port ${this.port}`);
            console.log(`📊 Health check: http://localhost:${this.port}/health`);
            console.log(`📈 Statistics: http://localhost:${this.port}/api/statistics`);
        });
    }

    stop() {
        console.log('🛑 C.A.R.E. API Server stopped');
    }
}

// Main execution
function main() {
    const api = new CareAPI(process.env.PORT || 3001);
    
    // Handle graceful shutdown
    process.on('SIGINT', () => {
        console.log('\n🛑 Shutting down C.A.R.E. API Server...');
        api.stop();
        process.exit(0);
    });
    
    process.on('SIGTERM', () => {
        console.log('\n🛑 Shutting down C.A.R.E. API Server...');
        api.stop();
        process.exit(0);
    });
    
    api.start();
}

// Run if this file is executed directly
if (require.main === module) {
    main();
}

module.exports = CareAPI;
