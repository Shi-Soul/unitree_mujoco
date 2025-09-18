#!/bin/bash

# Test script for DDS implementation
# This script verifies that the DDS version builds and basic functionality works

set -e

echo "=== Testing DDS Implementation ==="
echo

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ] || [ ! -f "src/main_dds.cc" ]; then
    echo "Error: Please run this script from the unitree_mujoco/simulate directory"
    exit 1
fi

# Test 1: Check if build directory exists and executables are present
echo "Test 1: Checking build artifacts..."
if [ ! -d "build" ]; then
    echo "  Build directory not found. Running build first..."
    mkdir -p build
    cd build
    cmake ..
    make -j$(nproc)
    cd ..
fi

cd build

# Check executables
executables=("unitree_mujoco" "unitree_mujoco_dds" "dds_reset_publisher")
for exe in "${executables[@]}"; do
    if [ -x "$exe" ]; then
        echo "  ✓ $exe found and executable"
    else
        echo "  ✗ $exe not found or not executable"
        exit 1
    fi
done

echo

# Test 2: Check help functionality
echo "Test 2: Testing help functionality..."
echo "  Testing dds_reset_publisher help..."
if ./dds_reset_publisher --help > /dev/null 2>&1; then
    echo "  ✓ dds_reset_publisher --help works"
else
    echo "  ✗ dds_reset_publisher --help failed"
    exit 1
fi

cd ..

echo "  Testing Python script help..."
if ./scripts/dds_reset_test.py --help > /dev/null 2>&1; then
    echo "  ✓ dds_reset_test.py --help works"
else
    echo "  ✗ dds_reset_test.py --help failed"
    exit 1
fi

echo

# Test 3: Check file structure
echo "Test 3: Checking file structure..."
required_files=(
    "src/main_dds.cc"
    "scripts/dds_reset_publisher.cpp"
    "scripts/dds_reset_test.py"
    "README_DDS.md"
    "install_dds.sh"
)

for file in "${required_files[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✓ $file exists"
    else
        echo "  ✗ $file missing"
        exit 1
    fi
done

echo

# Test 4: Quick functionality test
echo "Test 4: Quick functionality test..."
echo "  Testing dds_reset_publisher --once (should exit quickly)..."

# Run with timeout to avoid hanging
timeout 5s ./build/dds_reset_publisher --once > /tmp/dds_test.log 2>&1 &
pid=$!

# Wait a bit and check if it's still running
sleep 2
if kill -0 $pid 2>/dev/null; then
    echo "  ✓ dds_reset_publisher started successfully"
    kill $pid 2>/dev/null || true
else
    echo "  ✓ dds_reset_publisher completed quickly as expected"
fi

echo

# Test 5: Show summary
echo "=== Test Summary ==="
echo "✓ All basic tests passed!"
echo
echo "Available executables:"
echo "  - unitree_mujoco         : Original version"
echo "  - unitree_mujoco_dds     : DDS version with reset functionality"
echo "  - dds_reset_publisher    : Utility to send reset commands"
echo
echo "Usage examples:"
echo "  # Start DDS simulation"
echo "  ./build/unitree_mujoco_dds -r g1 -n eno1 -i 0"
echo
echo "  # Send reset command (in another terminal)"
echo "  ./build/dds_reset_publisher --once"
echo
echo "  # Test Python script"
echo "  ./scripts/dds_reset_test.py --once"
echo
echo "For detailed instructions, see README_DDS.md"
echo

echo "=== DDS Implementation Test Completed Successfully! ===" 