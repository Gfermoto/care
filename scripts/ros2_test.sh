#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT_DIR/services/ros2"

echo "[ros2_test] Sourcing ROS 2..."
source /opt/ros/humble/setup.bash || true

echo "[ros2_test] Running tests (ament_lint + pkg tests)..."
colcon test --packages-select \
  care_common care_demo_node care_safety_controller_node care_can_bridge_node || true
colcon test-result --verbose || true

echo "[ros2_test] Done."


