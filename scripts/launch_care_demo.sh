#!/bin/bash
# C.A.R.E. Demo Launch Script
# Запускает полную демонстрацию с Mock данными и RViz2

set -e

echo "🚀 C.A.R.E. Demo Launch Script"
echo "================================"

# Source ROS2
cd /home/gfer/CARE
set +u
source /opt/ros/jazzy/setup.bash
source services/ros2/install/setup.bash
set -u

# Kill old processes
echo "🧹 Cleaning old processes..."
killall -9 rviz2 2>/dev/null || true
pkill -f "can_bridge.py" 2>/dev/null || true
pkill -f "care_demo_node" 2>/dev/null || true
sleep 2

# Start CAN Bridge (Mock mode)
echo "📡 Starting CAN Bridge (Mock mode)..."
ros2 run care_can_bridge_node can_bridge.py --ros-args -p mode:=mock > /tmp/can_bridge.log 2>&1 &
CAN_PID=$!
sleep 2

# Start Demo Node (C++)
echo "🎯 Starting C.A.R.E. Demo Node..."
ros2 run care_demo_node care_demo_node > /tmp/demo_node.log 2>&1 &
DEMO_PID=$!
sleep 3

# Check if nodes are running
if ! ps -p $CAN_PID > /dev/null; then
    echo "❌ CAN Bridge failed to start"
    exit 1
fi

if ! ps -p $DEMO_PID > /dev/null; then
    echo "❌ Demo Node failed to start"
    exit 1
fi

# Show status
echo ""
echo "✅ Nodes started successfully!"
echo "   CAN Bridge PID: $CAN_PID"
echo "   Demo Node PID: $DEMO_PID"
echo ""

# Show topics
echo "📊 Active topics:"
ros2 topic list | grep care

echo ""
echo "🎨 Starting RViz2..."
rviz2 -d services/ros2/care_demo_node/config/care_demo.rviz > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!

echo ""
echo "✅ C.A.R.E. Demo started successfully!"
echo ""
echo "📊 You should see in RViz2:"
echo "   🎯 Animated targets (cascading appearance every 1 sec)"
echo "   🔵 Horizontal FOV (cyan, ±30°)"
echo "   🟢 Vertical FOV (green, ±17.5°)"
echo "   🔴 Safety zone (red cylinder, 1.5m radius)"
echo ""
echo "📋 Process IDs:"
echo "   CAN Bridge: $CAN_PID"
echo "   Demo Node: $DEMO_PID"
echo "   RViz2: $RVIZ_PID"
echo ""
echo "🛑 To stop: killall -9 rviz2 && pkill -f 'can_bridge|care_demo'"
echo ""
