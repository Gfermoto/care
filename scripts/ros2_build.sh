#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT_DIR/services/ros2"

echo "[ros2_build] Sourcing ROS 2..."
# Попробуем найти установленную версию ROS2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[ros2_build] Using ROS 2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[ros2_build] Using ROS 2 Humble"
else
    echo "[ros2_build] ERROR: ROS 2 not found!"
    exit 1
fi

echo "[ros2_build] Updating submodules..."
git -C "$ROOT_DIR" submodule update --init --recursive

echo "[ros2_build] Building selected packages with compile_commands.json..."
colcon build --merge-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  --packages-select \
  care_common care_demo_node care_safety_controller_node care_can_bridge_node

echo "[ros2_build] Creating symlink for compile_commands.json..."
if [ -f "$ROOT_DIR/services/ros2/build/compile_commands.json" ]; then
    ln -sf "$ROOT_DIR/services/ros2/build/compile_commands.json" "$ROOT_DIR/compile_commands.json"
    echo "[ros2_build] Symlink created at $ROOT_DIR/compile_commands.json"
fi

echo "[ros2_build] Done."


