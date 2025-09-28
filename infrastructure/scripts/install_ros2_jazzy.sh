#!/bin/bash

# C.A.R.E. - Установка ROS 2 Jazzy для Ubuntu 24.04
# Автор: C.A.R.E. Project
# Версия: 1.0.0

set -e  # Остановка при ошибке

# Цвета для вывода
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# Функция для вывода сообщений
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${PURPLE}$1${NC}"
}

# Проверка версии Ubuntu
check_ubuntu_version() {
    print_header "🔍 Проверка версии Ubuntu"
    
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        print_status "Обнаружена система: $PRETTY_NAME"
        
        if [[ "$VERSION_ID" == "24.04" ]]; then
            print_success "Ubuntu 24.04 - подходящая версия для ROS 2 Jazzy"
        else
            print_warning "Обнаружена версия: $VERSION_ID"
            print_status "Для Ubuntu 24.04 рекомендуется ROS 2 Jazzy"
        fi
    else
        print_warning "Не удалось определить версию системы"
    fi
}

# Установка системных зависимостей
install_system_dependencies() {
    print_header "📦 Установка системных зависимостей"
    
    print_status "Обновление пакетов..."
    sudo apt-get update -y
    
    print_status "Установка необходимых пакетов..."
    sudo apt-get install -y \
        software-properties-common \
        curl \
        gnupg \
        lsb-release \
        ca-certificates \
        build-essential \
        cmake \
        git \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool
    
    print_success "Системные зависимости установлены!"
}

# Настройка репозитория ROS 2
setup_ros2_repository() {
    print_header "🔧 Настройка репозитория ROS 2 Jazzy"
    
    print_status "Добавление ключа GPG..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    print_status "Добавление репозитория..."
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    print_status "Обновление списка пакетов..."
    sudo apt-get update -y
    
    print_success "Репозиторий ROS 2 Jazzy настроен!"
}

# Установка ROS 2 Jazzy
install_ros2_jazzy() {
    print_header "🚀 Установка ROS 2 Jazzy Desktop"
    
    print_status "Установка ROS 2 Jazzy Desktop (включает rviz2, rqt, демо)..."
    sudo apt-get install -y ros-jazzy-desktop
    
    print_status "Установка дополнительных пакетов для разработки..."
    sudo apt-get install -y \
        ros-jazzy-ament-cmake \
        ros-jazzy-ament-lint \
        ros-jazzy-ament-cmake-lint-cmake \
        ros-jazzy-ament-cmake-lint-cmake \
        ros-jazzy-ament-cmake-cppcheck \
        ros-jazzy-ament-cmake-cpplint \
        ros-jazzy-ament-cmake-flake8 \
        ros-jazzy-ament-cmake-pep257 \
        ros-jazzy-ament-cmake-uncrustify \
        ros-jazzy-ament-cmake-xmllint \
        ros-jazzy-ament-lint-auto \
        ros-jazzy-ament-lint-common \
        ros-jazzy-ament-lint-cmake \
        ros-jazzy-ament-lint-python \
        ros-jazzy-ament-cmake-clang-format \
        ros-jazzy-ament-cmake-clang-tidy \
        ros-jazzy-ament-cmake-copyright \
        ros-jazzy-ament-cmake-flake8 \
        ros-jazzy-ament-cmake-lint-cmake \
        ros-jazzy-ament-cmake-pep257 \
        ros-jazzy-ament-cmake-uncrustify \
        ros-jazzy-ament-cmake-xmllint \
        ros-jazzy-ament-lint-auto \
        ros-jazzy-ament-lint-common \
        ros-jazzy-ament-lint-cmake \
        ros-jazzy-ament-lint-python
    
    print_success "ROS 2 Jazzy установлен!"
}

# Настройка rosdep
setup_rosdep() {
    print_header "🔧 Настройка rosdep"
    
    print_status "Инициализация rosdep..."
    sudo rosdep init || true
    
    print_status "Обновление rosdep..."
    rosdep update
    
    print_success "rosdep настроен!"
}

# Настройка окружения
setup_environment() {
    print_header "🌍 Настройка окружения ROS 2"
    
    # Создание файла для автоматической настройки
    cat > /tmp/setup_ros2_env.sh << 'EOF'
#!/bin/bash
# Автоматическая настройка окружения ROS 2 Jazzy

# Source ROS 2 setup
source /opt/ros/jazzy/setup.bash

# Настройка переменных окружения
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

echo "✅ ROS 2 Jazzy окружение настроено!"
echo "🌍 ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "🔧 RMW: $RMW_IMPLEMENTATION"
EOF

    print_status "Создание скрипта настройки окружения..."
    sudo mv /tmp/setup_ros2_env.sh /usr/local/bin/setup_ros2_env.sh
    sudo chmod +x /usr/local/bin/setup_ros2_env.sh
    
    # Добавление в .bashrc
    print_status "Добавление в .bashrc..."
    if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "# ROS 2 Jazzy Setup" >> ~/.bashrc
        echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
        echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
        echo "export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp" >> ~/.bashrc
    fi
    
    print_success "Окружение ROS 2 настроено!"
}

# Установка дополнительных пакетов для C.A.R.E.
install_care_dependencies() {
    print_header "📦 Установка зависимостей для C.A.R.E."
    
    # Source ROS 2 для установки пакетов
    source /opt/ros/jazzy/setup.bash
    
    print_status "Установка пакетов для работы с CAN..."
    sudo apt-get install -y \
        can-utils \
        libsocketcan-dev \
        python3-can
    
    print_status "Установка дополнительных ROS 2 пакетов..."
    sudo apt-get install -y \
        ros-jazzy-cv-bridge \
        ros-jazzy-image-transport \
        ros-jazzy-tf2 \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-tf2-tools \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-xacro \
        ros-jazzy-urdf \
        ros-jazzy-rviz2 \
        ros-jazzy-rqt \
        ros-jazzy-rqt-common-plugins \
        ros-jazzy-rqt-robot-plugins \
        ros-jazzy-rqt-robot-steering \
        ros-jazzy-rqt-console \
        ros-jazzy-rqt-graph \
        ros-jazzy-rqt-plot \
        ros-jazzy-rqt-image-view \
        ros-jazzy-rqt-robot-monitor
    
    print_success "Зависимости C.A.R.E. установлены!"
}

# Проверка установки
verify_installation() {
    print_header "🔍 Проверка установки ROS 2 Jazzy"
    
    # Source ROS 2
    source /opt/ros/jazzy/setup.bash
    
    print_status "Проверка версии ROS 2..."
    if command -v ros2 &> /dev/null; then
        print_success "ROS 2 найден: $(ros2 --version)"
    else
        print_error "ROS 2 не найден!"
        exit 1
    fi
    
    print_status "Проверка доступных пакетов..."
    if ros2 pkg list | grep -q "ament_cmake"; then
        print_success "ROS 2 пакеты доступны"
    else
        print_warning "Некоторые пакеты могут быть недоступны"
    fi
    
    print_status "Проверка rclnodejs совместимости..."
    print_status "ROS 2 Jazzy совместим с rclnodejs 0.22.x"
    
    print_success "Установка ROS 2 Jazzy завершена успешно!"
}

# Информация о следующих шагах
show_next_steps() {
    print_header "🎯 Следующие шаги для C.A.R.E."
    
    echo ""
    print_success "🎉 ROS 2 Jazzy успешно установлен!"
    echo ""
    echo "📋 Для запуска C.A.R.E. выполните:"
    echo ""
    echo "1. 🔄 Перезагрузите терминал или выполните:"
    echo "   source ~/.bashrc"
    echo ""
    echo "2. 🚀 Запустите C.A.R.E. Node.js сервисы:"
    echo "   cd /home/gfer/CARE"
    echo "   chmod +x run.sh"
    echo "   ./run.sh"
    echo ""
    echo "3. 🔍 Проверьте ROS 2:"
    echo "   ros2 node list"
    echo "   ros2 topic list"
    echo ""
    echo "4. 📡 Настройте CAN интерфейс:"
    echo "   sudo ip link set can0 up type can bitrate 500000"
    echo ""
    echo "🌐 C.A.R.E. Web Dashboard: http://localhost:3000"
    echo "🔌 C.A.R.E. API Server: http://localhost:3001"
    echo ""
    print_success "Готово к работе с C.A.R.E.!"
}

# Основная функция
main() {
    print_header "🚀 C.A.R.E. - Установка ROS 2 Jazzy для Ubuntu 24.04"
    echo "========================================================="
    
    # Проверка версии Ubuntu
    check_ubuntu_version
    
    # Установка системных зависимостей
    install_system_dependencies
    
    # Настройка репозитория ROS 2
    setup_ros2_repository
    
    # Установка ROS 2 Jazzy
    install_ros2_jazzy
    
    # Настройка rosdep
    setup_rosdep
    
    # Настройка окружения
    setup_environment
    
    # Установка дополнительных пакетов
    install_care_dependencies
    
    # Проверка установки
    verify_installation
    
    # Информация о следующих шагах
    show_next_steps
}

# Запуск скрипта
main "$@"
