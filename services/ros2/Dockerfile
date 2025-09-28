# C.A.R.E. ROS 2 Docker Image
FROM ros:humble-desktop

# Установка системных зависимостей
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    curl \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Установка Python зависимостей
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

# Создание рабочей директории
WORKDIR /workspace

# Копирование ROS 2 пакетов
COPY . /workspace/

# Установка зависимостей ROS 2
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Компиляция ROS 2 пакетов
RUN colcon build

# Настройка окружения
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Expose ROS 2 порты
EXPOSE 11311 11312

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD ros2 node list || exit 1

# Запуск ROS 2
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch care_ld2450_driver care.launch.py"]
