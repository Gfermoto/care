#!/bin/bash

echo "🔧 C.A.R.E. - Установка ROS 2 Development пакетов"
echo "================================================="

# Настройка ROS 2 окружения
echo "🌍 Настройка ROS 2 окружения..."
set +u
source /opt/ros/jazzy/setup.bash

echo "📦 Установка ROS 2 development пакетов..."

# Установка основных пакетов для разработки
sudo apt-get update -y

# ROS 2 core development packages
sudo apt-get install -y \
    ros-jazzy-rclcpp-dev \
    ros-jazzy-rcl-dev \
    ros-jazzy-rcutils-dev \
    ros-jazzy-rmw-dev \
    ros-jazzy-rmw-implementation-dev \
    ros-jazzy-rmw-implementation-cmake \
    ros-jazzy-rmw-cyclonedx-cpp-dev \
    ros-jazzy-rosidl-dev \
    ros-jazzy-rosidl-runtime-c \
    ros-jazzy-rosidl-runtime-cpp \
    ros-jazzy-rosidl-typesupport-c \
    ros-jazzy-rosidl-typesupport-cpp \
    ros-jazzy-rosidl-typesupport-interface \
    ros-jazzy-rosidl-generator-c \
    ros-jazzy-rosidl-generator-cpp \
    ros-jazzy-rosidl-generator-dds-idl \
    ros-jazzy-rosidl-generator-py \
    ros-jazzy-rosidl-parser \
    ros-jazzy-rosidl-runtime-py \
    ros-jazzy-rosidl-typesupport-c-dev \
    ros-jazzy-rosidl-typesupport-cpp-dev \
    ros-jazzy-rosidl-typesupport-fastrtps-c \
    ros-jazzy-rosidl-typesupport-fastrtps-cpp \
    ros-jazzy-rosidl-typesupport-introspection-c \
    ros-jazzy-rosidl-typesupport-introspection-cpp

# Дополнительные пакеты для rclnodejs
sudo apt-get install -y \
    libboost-all-dev \
    python3-dev \
    python3-numpy \
    python3-setuptools \
    python3-wheel \
    python3-pip \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libffi-dev \
    libxml2-dev \
    libxslt1-dev \
    zlib1g-dev \
    libjpeg-dev \
    libpng-dev

echo "✅ ROS 2 development пакеты установлены!"

# Проверка установки
echo "🔍 Проверка установки..."
if command -v ros2 &> /dev/null; then
    echo "✅ ROS 2 command line tools: OK"
else
    echo "❌ ROS 2 command line tools: FAILED"
fi

# Проверка заголовочных файлов
if [ -f "/opt/ros/jazzy/include/rcl/rcl.h" ]; then
    echo "✅ RCL headers: OK"
else
    echo "❌ RCL headers: MISSING"
fi

if [ -f "/opt/ros/jazzy/include/rclcpp/rclcpp.hpp" ]; then
    echo "✅ RCLCPP headers: OK"
else
    echo "❌ RCLCPP headers: MISSING"
fi

echo ""
echo "🎉 Установка завершена!"
echo "🚀 Теперь можно запустить C.A.R.E.:"
echo "   ./start.sh"
