#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT_DIR/services/ros2"

echo "[ros2_build] Sourcing ROS 2..."
source /opt/ros/humble/setup.bash || true

echo "[ros2_build] Updating submodules..."
git -C "$ROOT_DIR" submodule update --init --recursive

echo "[ros2_build] Building selected packages..."
colcon build --merge-install --packages-select \
  care_common care_demo_node care_safety_controller_node care_can_bridge_node

echo "[ros2_build] Done."


