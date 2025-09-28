# 🚀 C.A.R.E. - Руководство по развертыванию

## 📋 Обзор

Данное руководство описывает процесс развертывания системы C.A.R.E. в различных средах: разработка, тестирование и продакшен.

## 🎯 Требования к системе

### Минимальные требования
- **ОС**: Ubuntu 20.04+ или Debian 11+
- **RAM**: 4GB (рекомендуется 8GB)
- **CPU**: 2 ядра (рекомендуется 4 ядра)
- **Диск**: 20GB свободного места
- **Сеть**: Ethernet для CAN интерфейса

### Рекомендуемые требования
- **ОС**: Ubuntu 24.04 LTS
- **RAM**: 16GB
- **CPU**: 8 ядер
- **Диск**: 50GB SSD
- **Сеть**: Gigabit Ethernet + WiFi

## 🔧 Предварительная настройка

### 1. Системные зависимости
```bash
# Обновление системы
sudo apt update && sudo apt upgrade -y

# Установка базовых пакетов
sudo apt install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    vim \
    htop \
    can-utils \
    libsocketcan-dev

# Установка Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Установка Docker Compose
sudo curl -L "https://github.com/docker/compose/releases/latest/download/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose
```

### 2. Настройка CAN интерфейса
```bash
# Загрузка CAN модулей
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan

# Создание виртуального CAN интерфейса (для тестирования)
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Настройка физического CAN интерфейса
sudo ip link set can0 up type can bitrate 500000
```

### 3. Клонирование проекта
```bash
# Клонирование репозитория
git clone https://github.com/your-org/care-system.git
cd care-system

# Установка прав доступа
chmod +x infrastructure/scripts/*.sh
chmod +x start.sh
```

## 🛠️ Развертывание для разработки

### 1. Конфигурация датчика
```bash
# Выбор типа датчика (mock для разработки)
./infrastructure/scripts/configure_sensor.sh
# Выберите: 1) Mock CAN
```

### 2. Установка ROS2 Jazzy
```bash
# Автоматическая установка ROS2
./infrastructure/scripts/install_ros2_jazzy.sh

# Проверка установки
source /opt/ros/jazzy/setup.bash
ros2 --version
```

### 3. Установка Node.js зависимостей
```bash
# Установка Node.js 18.x
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs

# Проверка версий
node --version  # должно быть v18.x+
npm --version   # должно быть v9.x+
```

### 4. Запуск в режиме разработки
```bash
# Экспорт переменных окружения
export CARE_SENSOR_TYPE=mock
export ROS_DOMAIN_ID=0

# Запуск системы
./start.sh
```

### 5. Проверка работоспособности
```bash
# Проверка веб-интерфейса
curl http://localhost:3000

# Проверка API
curl http://localhost:3001/health

# Проверка ROS2
ros2 topic list
ros2 node list

# Мониторинг Mock CAN
./infrastructure/scripts/test_mock_can.sh
```

## 🐳 Развертывание в контейнерах

### 1. Подготовка окружения
```bash
# Создание директорий для данных
sudo mkdir -p /opt/care/{data,logs,configs}
sudo chown -R $USER:$USER /opt/care

# Копирование конфигурационных файлов
cp infrastructure/configs/* /opt/care/configs/
```

### 2. Конфигурация Docker Compose
```bash
# Переход в директорию Docker
cd infrastructure/docker

# Настройка переменных окружения
cp .env.example .env
vim .env  # Отредактируйте переменные
```

Пример `.env` файла:
```bash
# Sensor Configuration
CARE_SENSOR_TYPE=real
CAN_INTERFACE=can0
CAN_BITRATE=500000

# ROS2 Configuration
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Node.js Configuration
NODE_ENV=production
API_PORT=3001
DASHBOARD_PORT=3000

# Redis Configuration
REDIS_PASSWORD=your_secure_password

# Nginx Configuration
SSL_CERT_PATH=/opt/care/ssl/cert.pem
SSL_KEY_PATH=/opt/care/ssl/key.pem
```

### 3. Сборка контейнеров
```bash
# Сборка всех сервисов
docker-compose build

# Проверка образов
docker images | grep care
```

### 4. Запуск в продакшене
```bash
# Запуск всех сервисов
docker-compose up -d

# Проверка статуса
docker-compose ps

# Просмотр логов
docker-compose logs -f care-nodejs
```

## 🔒 Настройка безопасности

### 1. SSL/TLS сертификаты
```bash
# Создание самоподписанного сертификата (для тестирования)
sudo mkdir -p /opt/care/ssl
sudo openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
    -keyout /opt/care/ssl/key.pem \
    -out /opt/care/ssl/cert.pem

# Или использование Let's Encrypt (для продакшена)
sudo apt install certbot
sudo certbot certonly --standalone -d your-domain.com
```

### 2. Firewall настройка
```bash
# Установка UFW
sudo apt install ufw

# Базовые правила
sudo ufw default deny incoming
sudo ufw default allow outgoing

# Разрешение необходимых портов
sudo ufw allow 22/tcp      # SSH
sudo ufw allow 80/tcp      # HTTP
sudo ufw allow 443/tcp     # HTTPS
sudo ufw allow 3000/tcp    # Dashboard
sudo ufw allow 3001/tcp    # API

# Активация firewall
sudo ufw enable
```

### 3. Пользователи и права доступа
```bash
# Создание пользователя для сервиса
sudo useradd -r -s /bin/false care-service
sudo usermod -aG docker care-service

# Настройка прав доступа
sudo chown -R care-service:care-service /opt/care
sudo chmod -R 750 /opt/care
```

## 📊 Мониторинг и логирование

### 1. Настройка логирования
```bash
# Конфигурация rsyslog для C.A.R.E.
sudo tee /etc/rsyslog.d/50-care.conf << EOF
# C.A.R.E. System Logs
local0.*    /var/log/care/system.log
local1.*    /var/log/care/ros2.log
local2.*    /var/log/care/nodejs.log
EOF

# Создание директории логов
sudo mkdir -p /var/log/care
sudo chown syslog:adm /var/log/care

# Перезапуск rsyslog
sudo systemctl restart rsyslog
```

### 2. Ротация логов
```bash
# Конфигурация logrotate
sudo tee /etc/logrotate.d/care << EOF
/var/log/care/*.log {
    daily
    missingok
    rotate 30
    compress
    delaycompress
    notifempty
    create 644 syslog adm
    postrotate
        systemctl reload rsyslog
    endscript
}
EOF
```

### 3. Health checks
```bash
# Создание скрипта мониторинга
sudo tee /usr/local/bin/care-health-check.sh << 'EOF'
#!/bin/bash

HEALTH_LOG="/var/log/care/health.log"
DATE=$(date '+%Y-%m-%d %H:%M:%S')

# Проверка Node.js API
if curl -sf http://localhost:3001/health > /dev/null; then
    echo "[$DATE] API: OK" >> $HEALTH_LOG
else
    echo "[$DATE] API: FAILED" >> $HEALTH_LOG
fi

# Проверка ROS2
if docker exec care-ros2 ros2 node list > /dev/null 2>&1; then
    echo "[$DATE] ROS2: OK" >> $HEALTH_LOG
else
    echo "[$DATE] ROS2: FAILED" >> $HEALTH_LOG
fi

# Проверка Redis
if docker exec care-redis redis-cli ping > /dev/null 2>&1; then
    echo "[$DATE] Redis: OK" >> $HEALTH_LOG
else
    echo "[$DATE] Redis: FAILED" >> $HEALTH_LOG
fi
EOF

sudo chmod +x /usr/local/bin/care-health-check.sh

# Добавление в crontab
echo "*/5 * * * * /usr/local/bin/care-health-check.sh" | sudo crontab -
```

## 🔄 Обновление и обслуживание

### 1. Обновление системы
```bash
# Остановка сервисов
docker-compose down

# Резервное копирование данных
sudo tar -czf /opt/backups/care-$(date +%Y%m%d).tar.gz /opt/care/data

# Обновление кода
git pull origin main

# Пересборка контейнеров
docker-compose build

# Запуск обновленных сервисов
docker-compose up -d
```

### 2. Резервное копирование
```bash
# Скрипт резервного копирования
sudo tee /usr/local/bin/care-backup.sh << 'EOF'
#!/bin/bash

BACKUP_DIR="/opt/backups"
DATE=$(date +%Y%m%d_%H%M%S)
BACKUP_FILE="care_backup_$DATE.tar.gz"

# Создание директории backup
mkdir -p $BACKUP_DIR

# Резервное копирование данных
tar -czf $BACKUP_DIR/$BACKUP_FILE \
    /opt/care/data \
    /opt/care/configs \
    /var/log/care

# Удаление старых backup (старше 30 дней)
find $BACKUP_DIR -name "care_backup_*.tar.gz" -mtime +30 -delete

echo "Backup created: $BACKUP_FILE"
EOF

sudo chmod +x /usr/local/bin/care-backup.sh

# Ежедневное резервное копирование
echo "0 2 * * * /usr/local/bin/care-backup.sh" | sudo crontab -
```

## 🐛 Troubleshooting

### Общие проблемы

#### 1. CAN интерфейс не работает
```bash
# Проверка CAN интерфейса
ip link show can0

# Перезапуск CAN интерфейса
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000

# Тестирование CAN
cansend can0 123#DEADBEEF
candump can0
```

#### 2. ROS2 не запускается
```bash
# Проверка ROS2 окружения
echo $ROS_DISTRO
source /opt/ros/jazzy/setup.bash

# Проверка ROS2 демона
ros2 daemon stop
ros2 daemon start

# Проверка топиков
ros2 topic list
```

#### 3. Node.js сервисы недоступны
```bash
# Проверка портов
sudo netstat -tulpn | grep :300

# Проверка логов
docker logs care-nodejs

# Перезапуск сервиса
docker-compose restart care-nodejs
```

#### 4. Docker контейнеры не запускаются
```bash
# Проверка статуса
docker-compose ps

# Проверка логов
docker-compose logs

# Очистка и пересборка
docker-compose down
docker system prune -f
docker-compose build --no-cache
docker-compose up -d
```

### Диагностические команды

```bash
# Системная информация
./infrastructure/scripts/system-info.sh

# Проверка всех сервисов
./infrastructure/scripts/health-check.sh

# Сбор логов для отладки
./infrastructure/scripts/collect-logs.sh

# Тест производительности
./infrastructure/scripts/performance-test.sh
```

## 📞 Поддержка

При возникновении проблем:

1. **Проверьте логи**: `/var/log/care/`
2. **Запустите диагностику**: `./infrastructure/scripts/health-check.sh`
3. **Соберите информацию**: `./infrastructure/scripts/collect-logs.sh`
4. **Создайте issue**: В репозитории проекта с приложенными логами

---

**Успешного развертывания C.A.R.E. системы!** 🚀
