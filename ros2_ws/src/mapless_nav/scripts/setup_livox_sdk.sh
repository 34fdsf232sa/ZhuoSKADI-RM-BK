#!/bin/bash
# =============================================================================
# Livox SDK2 + livox_ros_driver2 安装脚本
# 用于 Mid-70 激光雷达的 ROS2 Humble 驱动安装
# =============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="$SCRIPT_DIR/ros2_ws"
LIVOX_SDK_DIR="$HOME/Livox-SDK2"

echo "=== Livox SDK2 + ROS2 Driver 安装 ==="
echo ""

# Step 1: Install Livox SDK2
echo "[1/3] 安装 Livox SDK2..."
if [ -d "$LIVOX_SDK_DIR" ]; then
    echo "  SDK2 already exists at $LIVOX_SDK_DIR, pulling latest..."
    cd "$LIVOX_SDK_DIR" && git pull
else
    git clone https://github.com/Livox-SDK/Livox-SDK2.git "$LIVOX_SDK_DIR"
    cd "$LIVOX_SDK_DIR"
fi

if [ ! -d "$LIVOX_SDK_DIR/build" ]; then
    mkdir -p "$LIVOX_SDK_DIR/build"
fi
cd "$LIVOX_SDK_DIR/build"
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
echo "  SDK2 安装完成"

# Step 2: Check livox_ros_driver2 in workspace
echo ""
echo "[2/3] 检查 livox_ros_driver2..."
cd "$ROS2_WS"
if [ -d "src/livox_ros_driver2" ]; then
    echo "  livox_ros_driver2 已存在于 src/ 目录"
else
    echo "  正在克隆 livox_ros_driver2..."
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2
fi

# Step 3: Build
echo ""
echo "[3/3] 编译 livox_ros_driver2..."
source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2 --symlink-install
echo ""

echo "=== 安装完成 ==="
echo ""
echo "使用方法:"
echo "  source $ROS2_WS/install/setup.bash"
echo "  ros2 launch livox_ros_driver2 msg_MID70_launch.py"
echo ""
echo "话题: /livox/lidar (sensor_msgs/PointCloud2)"
