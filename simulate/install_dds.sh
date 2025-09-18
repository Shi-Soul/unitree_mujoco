#!/bin/bash

# MuJoCo DDS Version Installation Script
# This script builds and installs the DDS version of unitree_mujoco

set -e

echo "=== MuJoCo DDS Version Installation ==="
echo "This script will build and install the DDS version with reset functionality"
echo

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ] || [ ! -f "src/main_dds.cc" ]; then
    echo "Error: Please run this script from the unitree_mujoco/simulate directory"
    echo "Current directory: $(pwd)"
    exit 1
fi

# Check for required dependencies
echo "Checking dependencies..."

# Check for unitree_sdk2
if ! pkg-config --exists unitree_sdk2 2>/dev/null && [ ! -d "/opt/unitree_robotics" ]; then
    echo "Warning: Unitree SDK2 not found. Please ensure it's installed."
fi

# Check for mujoco
if ! pkg-config --exists mujoco 2>/dev/null; then
    echo "Warning: MuJoCo not found via pkg-config. Please ensure it's installed."
fi

echo "Dependencies check completed."
echo

# Create build directory
echo "Creating build directory..."
mkdir -p build
cd build

echo "Configuring with CMake..."
cmake ..

echo "Building (using $(nproc) cores)..."
make -j$(nproc)

echo
echo "=== Build completed successfully! ==="
echo
echo "Generated executables:"
echo "  - unitree_mujoco         : Original version"
echo "  - unitree_mujoco_dds     : DDS version with reset functionality"
echo "  - dds_reset_publisher    : Utility to send reset commands"
echo

# Optional installation to system
read -p "Do you want to install to system directories? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Installing to system..."
    sudo make install
    echo "Installation completed!"
    echo
    echo "You can now run:"
    echo "  unitree_mujoco_dds -r g1 -n eno1 -i 0"
    echo "  dds_reset_publisher --once"
else
    echo "Not installing to system. You can run executables from:"
    echo "  $(pwd)/unitree_mujoco_dds"
    echo "  $(pwd)/dds_reset_publisher"
fi

echo
echo "=== Quick Start Guide ==="
echo
echo "1. Start the DDS simulation:"
echo "   ./unitree_mujoco_dds -r g1 -n eno1 -i 0"
echo
echo "2. In another terminal, send a reset command:"
echo "   ./dds_reset_publisher --once"
echo
echo "3. For continuous resets every 10 seconds:"
echo "   ./dds_reset_publisher --loop"
echo
echo "For more information, see README_DDS.md"
echo
echo "Installation completed successfully!" 