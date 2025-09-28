#!/usr/bin/env node

/**
 * C.A.R.E. Node.js Main Entry Point
 * 
 * Orchestrates all Node.js services for the C.A.R.E. radar system
 * - CAN Bridge (CAN ↔ ROS 2)
 * - Web Dashboard (Real-time monitoring)
 * - API Server (Configuration and data access)
 */

const CareCANBridge = require('./care-can-bridge');
const CareDashboard = require('./care-dashboard');
const CareAPI = require('./care-api');

class CareNodeJS {
    constructor() {
        this.services = {
            canBridge: null,
            dashboard: null,
            api: null
        };
        
        this.isRunning = false;
    }

    async start() {
        try {
            console.log('🚀 Starting C.A.R.E. Node.js Services...');
            
            // Start API server first (no dependencies)
            console.log('📊 Starting API Server...');
            this.services.api = new CareAPI(3001);
            this.services.api.start();
            
            // Start Web Dashboard
            console.log('🌐 Starting Web Dashboard...');
            this.services.dashboard = new CareDashboard(3000);
            this.services.dashboard.start();
            
            // Start CAN Bridge (depends on ROS 2)
            console.log('📡 Starting CAN Bridge...');
            this.services.canBridge = new CareCANBridge();
            await this.services.canBridge.start();
            
            this.isRunning = true;
            
            console.log('🎉 All C.A.R.E. Node.js services started successfully!');
            console.log('📊 Web Dashboard: http://localhost:3000');
            console.log('🔌 API Server: http://localhost:3001');
            console.log('📡 CAN Bridge: Connected to ROS 2');
            
        } catch (error) {
            console.error('❌ Failed to start C.A.R.E. services:', error);
            await this.stop();
            process.exit(1);
        }
    }

    async stop() {
        try {
            console.log('🛑 Stopping C.A.R.E. Node.js services...');
            
            if (this.services.canBridge) {
                await this.services.canBridge.stop();
            }
            
            if (this.services.dashboard) {
                this.services.dashboard.stop();
            }
            
            if (this.services.api) {
                this.services.api.stop();
            }
            
            this.isRunning = false;
            console.log('✅ All C.A.R.E. services stopped');
            
        } catch (error) {
            console.error('❌ Error stopping services:', error);
        }
    }

    getStatus() {
        return {
            running: this.isRunning,
            services: {
                canBridge: this.services.canBridge ? 'running' : 'stopped',
                dashboard: this.services.dashboard ? 'running' : 'stopped',
                api: this.services.api ? 'running' : 'stopped'
            },
            timestamp: new Date().toISOString()
        };
    }
}

// Main execution
async function main() {
    const care = new CareNodeJS();
    
    // Handle graceful shutdown
    process.on('SIGINT', async () => {
        console.log('\n🛑 Shutting down C.A.R.E. Node.js services...');
        await care.stop();
        process.exit(0);
    });
    
    process.on('SIGTERM', async () => {
        console.log('\n🛑 Shutting down C.A.R.E. Node.js services...');
        await care.stop();
        process.exit(0);
    });
    
    // Handle uncaught exceptions
    process.on('uncaughtException', async (error) => {
        console.error('❌ Uncaught Exception:', error);
        await care.stop();
        process.exit(1);
    });
    
    process.on('unhandledRejection', async (reason, promise) => {
        console.error('❌ Unhandled Rejection at:', promise, 'reason:', reason);
        await care.stop();
        process.exit(1);
    });
    
    await care.start();
}

// Run if this file is executed directly
if (require.main === module) {
    main().catch(error => {
        console.error('❌ Fatal error:', error);
        process.exit(1);
    });
}

module.exports = CareNodeJS;
