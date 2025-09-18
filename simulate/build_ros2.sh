#!/bin/bash

set -e

echo "=== Building MuJoCo Simulation with ROS2 Support ==="
echo

# Check if we're in the correct directory
if [ ! -f "CMakeLists.txt" ]; then
    echo "Error: CMakeLists.txt not found. Please run this script from the simulate directory."
    exit 1
fi

# Create build directory
echo "Creating build directory..."
mkdir -p build
cd build

# Configure with CMake
echo "Configuring with CMake..."
cmake .. -DBUILD_ROS2=ON -DCMAKE_BUILD_TYPE=Release

# Build
echo "Building..."
make -j$(nproc)

echo
echo "=== Build Complete ==="
echo
echo "Executables created:"
echo "  - unitree_mujoco     (original version)"
echo "  - unitree_mujoco_ros (ROS2 version with reset support)"
echo
echo "To test the ROS2 version:"
echo "  1. Start the simulator: ./unitree_mujoco_ros -r g1 -n eno1 -i 0"
echo "  2. In another terminal, send reset: ros2 topic pub --once /mjc/reset std_msgs/msg/Empty"
echo
echo "See README_ROS2.md for detailed usage instructions." 