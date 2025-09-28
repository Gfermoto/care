# RViz2 Configuration for C.A.R.E. Project

## Overview
RViz2 is the 3D visualization tool for ROS 2, perfect for monitoring C.A.R.E. radar data and safety zones.

## Installation Status
✅ **RViz2 is installed** (version 14.1.14) with ROS 2 Jazzy

## Launch RViz2
```bash
# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Launch RViz2
rviz2

# Launch with custom config
rviz2 -d ~/care_radar.rviz
```

## C.A.R.E. Topics for Visualization

### ESP32 Topics
- `/care/radar_targets` - LD2450 detected targets
- `/care/safety_status` - Safety zone status
- `/care/wifi_config` - WiFi configuration status

### STM32 Topics  
- `/care/emergency_stop` - Emergency stop signals
- `/care/target_data` - Processed target data
- `/care/can_status` - CAN bus status

## Recommended RViz2 Plugins

### For Radar Data
1. **PointCloud2** - Display radar points
2. **Marker** - Show target positions
3. **MarkerArray** - Multiple targets
4. **Polygon** - Safety zone boundaries

### For Robot Integration
1. **TF** - Coordinate transformations
2. **RobotModel** - Robot visualization
3. **LaserScan** - If using laser data
4. **Map** - Environment mapping

## Sample RViz2 Configuration

### Fixed Frame
- Set to `base_link` or `radar_link`

### Displays to Add
1. **Add → By topic → /care/radar_targets → PointCloud2**
2. **Add → By topic → /care/safety_zone → Polygon**
3. **Add → By topic → /care/emergency_stop → Marker**
4. **Add → By display type → TF**

### View Setup
- **Target Frame**: `base_link`
- **Camera Type**: Orbit
- **Distance**: 5.0m
- **Pitch**: 0.5 rad
- **Yaw**: 0.0 rad

## Color Coding
- **Green**: Safe targets
- **Yellow**: Warning zone
- **Red**: Emergency stop zone
- **Blue**: Robot base/sensors

## Integration with Node.js Dashboard
RViz2 complements the Node.js web dashboard:
- **RViz2**: 3D visualization, detailed analysis
- **Web Dashboard**: Real-time monitoring, configuration

## Troubleshooting

### RViz2 won't start
```bash
# Check ROS 2 environment
echo $ROS_DISTRO

# Source environment
source /opt/ros/jazzy/setup.bash

# Check display
echo $DISPLAY
```

### No data in RViz2
```bash
# Check topics
ros2 topic list

# Check topic data
ros2 topic echo /care/radar_targets

# Check transforms
ros2 run tf2_tools view_frames.py
```

## Advanced Features

### Recording Data
```bash
# Record all C.A.R.E. topics
ros2 bag record /care/*

# Playback recorded data
ros2 bag play care_data.bag
```

### Custom Plugins
Create custom RViz2 plugins for specialized C.A.R.E. visualizations.

## Performance Tips
- Limit point cloud size for better performance
- Use appropriate update rates
- Close unused displays
- Adjust quality settings for slower systems
