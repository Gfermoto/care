const express = require('express');
const cors = require('cors');
const bodyParser = require('body-parser');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const WebSocket = require('ws');
const winston = require('winston');

// Настройка логирования
const logger = winston.createLogger({
  level: 'info',
  format: winston.format.combine(
    winston.format.timestamp(),
    winston.format.json()
  ),
  transports: [
    new winston.transports.Console(),
    new winston.transports.File({ filename: 'stm32-config.log' })
  ]
});

const app = express();
const PORT = process.env.PORT || 3002;

// Middleware
app.use(cors());
app.use(bodyParser.json());
app.use(express.static('public'));

// Конфигурация STM32
let stm32Config = {
  can: {
    bitrate: 500000,
    txPin: 'PB0',
    rxPin: 'PB1'
  },
  radar: {
    uartPort: 'UART2',
    baudRate: 256000,
    maxTargets: 3,
    detectionRange: 8000,
    updateRate: 20
  },
  safety: {
    emergencyRadius: 1000,
    emergencyAngle: 30,
    reactionTime: 50
  }
};

// UART соединение с STM32
let stm32Port = null;
let stm32Parser = null;

// WebSocket для реального времени
const wss = new WebSocket.Server({ port: 8082 });

// Инициализация UART соединения
function initSTM32Connection() {
  try {
    // Поиск STM32 устройства
    const devices = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1'];
    
    for (const device of devices) {
      try {
        stm32Port = new SerialPort({
          path: device,
          baudRate: 115200, // Скорость для конфигурации
          autoOpen: false
        });
        
        stm32Parser = stm32Port.pipe(new ReadlineParser({ delimiter: '\r\n' }));
        
        stm32Port.open((err) => {
          if (err) {
            logger.error(`Ошибка открытия ${device}: ${err.message}`);
            return;
          }
          
          logger.info(`STM32 подключен: ${device}`);
          
          // Обработка данных от STM32
          stm32Parser.on('data', (data) => {
            logger.info(`STM32: ${data}`);
            broadcastToClients({ type: 'stm32_data', data: data });
          });
          
          return; // Успешно подключились
        });
        
        break;
      } catch (err) {
        logger.warn(`Не удалось подключиться к ${device}: ${err.message}`);
      }
    }
  } catch (err) {
    logger.error(`Ошибка инициализации STM32: ${err.message}`);
  }
}

// WebSocket broadcast
function broadcastToClients(message) {
  wss.clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(message));
    }
  });
}

// API Routes

// Получение конфигурации STM32
app.get('/api/stm32/config', (req, res) => {
  res.json({
    success: true,
    config: stm32Config,
    connected: stm32Port ? stm32Port.isOpen : false
  });
});

// Настройка CAN параметров
app.post('/api/stm32/can', (req, res) => {
  try {
    const { bitrate, txPin, rxPin } = req.body;
    
    stm32Config.can = { bitrate, txPin, rxPin };
    
    // Отправка команды в STM32
    if (stm32Port && stm32Port.isOpen) {
      const command = `CAN_CONFIG:${bitrate}:${txPin}:${rxPin}\r\n`;
      stm32Port.write(command);
      logger.info(`CAN настроен: ${command}`);
    }
    
    res.json({
      success: true,
      message: 'CAN параметры обновлены',
      config: stm32Config.can
    });
  } catch (err) {
    res.status(500).json({
      success: false,
      message: err.message
    });
  }
});

// Настройка радара
app.post('/api/stm32/radar', (req, res) => {
  try {
    const { uartPort, baudRate, maxTargets, detectionRange, updateRate } = req.body;
    
    stm32Config.radar = { uartPort, baudRate, maxTargets, detectionRange, updateRate };
    
    // Отправка команды в STM32
    if (stm32Port && stm32Port.isOpen) {
      const command = `RADAR_CONFIG:${uartPort}:${baudRate}:${maxTargets}:${detectionRange}:${updateRate}\r\n`;
      stm32Port.write(command);
      logger.info(`Радар настроен: ${command}`);
    }
    
    res.json({
      success: true,
      message: 'Параметры радара обновлены',
      config: stm32Config.radar
    });
  } catch (err) {
    res.status(500).json({
      success: false,
      message: err.message
    });
  }
});

// Настройка безопасности
app.post('/api/stm32/safety', (req, res) => {
  try {
    const { emergencyRadius, emergencyAngle, reactionTime } = req.body;
    
    stm32Config.safety = { emergencyRadius, emergencyAngle, reactionTime };
    
    // Отправка команды в STM32
    if (stm32Port && stm32Port.isOpen) {
      const command = `SAFETY_CONFIG:${emergencyRadius}:${emergencyAngle}:${reactionTime}\r\n`;
      stm32Port.write(command);
      logger.info(`Безопасность настроена: ${command}`);
    }
    
    res.json({
      success: true,
      message: 'Параметры безопасности обновлены',
      config: stm32Config.safety
    });
  } catch (err) {
    res.status(500).json({
      success: false,
      message: err.message
    });
  }
});

// Отладка через COM порт
app.post('/api/stm32/debug', (req, res) => {
  try {
    const { command } = req.body;
    
    if (!stm32Port || !stm32Port.isOpen) {
      return res.status(400).json({
        success: false,
        message: 'STM32 не подключен'
      });
    }
    
    // Отправка команды отладки
    stm32Port.write(`${command}\r\n`);
    logger.info(`Отладочная команда: ${command}`);
    
    res.json({
      success: true,
      message: 'Команда отправлена',
      command: command
    });
  } catch (err) {
    res.status(500).json({
      success: false,
      message: err.message
    });
  }
});

// Мониторинг в реальном времени
app.get('/api/stm32/monitor', (req, res) => {
  res.json({
    success: true,
    connected: stm32Port ? stm32Port.isOpen : false,
    config: stm32Config,
    uptime: process.uptime()
  });
});

// WebSocket соединения
wss.on('connection', (ws) => {
  logger.info('WebSocket клиент подключен');
  
  ws.on('message', (message) => {
    try {
      const data = JSON.parse(message);
      logger.info(`WebSocket сообщение: ${data}`);
    } catch (err) {
      logger.error(`Ошибка парсинга WebSocket: ${err.message}`);
    }
  });
  
  ws.on('close', () => {
    logger.info('WebSocket клиент отключен');
  });
});

// Запуск сервера
app.listen(PORT, () => {
  logger.info(`C.A.R.E. STM32 Config Service запущен на порту ${PORT}`);
  logger.info(`WebSocket сервер на порту 8082`);
  
  // Инициализация STM32 соединения
  initSTM32Connection();
});

// Graceful shutdown
process.on('SIGINT', () => {
  logger.info('Завершение работы...');
  
  if (stm32Port && stm32Port.isOpen) {
    stm32Port.close();
  }
  
  process.exit(0);
});
