# MuJoCo Simulation with DDS Reset Support

This version provides robot reset functionality using Unitree SDK2's native DDS implementation, avoiding conflicts with ROS2's DDS library.

## Features

- **Original MuJoCo Simulation**: All original functionality is preserved
- **Native DDS Integration**: Uses Unitree SDK2's DDS for reset commands
- **No ROS2 Conflicts**: Avoids DDS version conflicts with ROS2 Foxy
- **Reset Command**: Subscribe to `mjc/reset` DDS topic to reset robot to default position
- **Thread-safe**: Reset commands are processed safely in the physics loop

## Building

### Prerequisites

- MuJoCo library
- Unitree SDK2
- All original dependencies (glfw, yaml-cpp, boost, fmt)
- **No ROS2 required**

### Build Instructions

```bash
# Navigate to the simulate directory
cd src/unitree_mujoco/simulate

# Build with standard CMake (no special flags needed)
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

This will create:
- `unitree_mujoco` - Original version (unchanged)
- `unitree_mujoco_dds` - DDS version with reset functionality
- `dds_reset_publisher` - Utility to send reset commands

## Usage

### 1. Start the DDS MuJoCo Simulation

```bash
# Navigate to build directory
cd src/unitree_mujoco/simulate/build

# Start the DDS version
./unitree_mujoco_dds -r g1 -n eno1 -i 0

# Or for Go2
./unitree_mujoco_dds -r go2 -n eno1 -i 0
```

### 2. Send Reset Commands

#### Option A: Using the DDS reset publisher utility

```bash
# Send a single reset command
./dds_reset_publisher --once

# Send continuous reset commands every 10 seconds
./dds_reset_publisher --loop

# Show help
./dds_reset_publisher --help
```

#### Option B: Programmatic usage

Create your own DDS publisher using the same message structure:

```cpp
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

struct ResetMessage
{
    uint32_t timestamp;
    bool reset_flag;
};

// Initialize DDS
unitree::robot::ChannelFactory::Instance()->Init(0, "");

// Create publisher
auto reset_pub = std::make_unique<unitree::robot::ChannelPublisher<ResetMessage>>("mjc/reset");

// Send reset command
ResetMessage msg;
msg.timestamp = getCurrentTimestamp();
msg.reset_flag = true;
reset_pub->Write(msg);
```

## Reset Functionality

When a reset command is received:

1. **Position Reset**: Robot joints return to default positions (keyframe 0 if available, otherwise `qpos0`)
2. **Velocity Reset**: All joint velocities are set to zero
3. **Force Reset**: All applied external forces are cleared
4. **Forward Kinematics**: `mj_forward()` is called to update derived quantities

## DDS Topic Interface

| Topic       | Message Type   | Description                     |
| ----------- | -------------- | ------------------------------- |
| `mjc/reset` | `ResetMessage` | Reset robot to default position |

### ResetMessage Structure

```cpp
struct ResetMessage
{
    uint32_t timestamp;  // Timestamp when command was sent
    bool reset_flag;     // Set to true to trigger reset
};
```

## Files Added/Modified

### Core Files
- `src/main_dds.cc` - DDS version of main.cc with reset functionality
- `scripts/dds_reset_publisher.cpp` - Utility to send reset commands
- `CMakeLists.txt` - Updated to build DDS version

### Key Changes in main_dds.cc

1. **Added DDS includes**:
   ```cpp
   #include <unitree/robot/channel/channel_subscriber.hpp>
   #include <unitree/robot/channel/channel_factory.hpp>
   ```

2. **Added reset message structure**:
   ```cpp
   struct ResetMessage {
       uint32_t timestamp;
       bool reset_flag;
   };
   ```

3. **Added reset subscriber class**:
   ```cpp
   class ResetSubscriber {
       // Subscribes to mjc/reset topic
   };
   ```

4. **Added reset check in physics loop**:
   ```cpp
   if (reset_requested.load()) {
       ResetRobot();
       reset_requested.store(false);
   }
   ```

## Advantages over ROS2 Version

1. **No DDS Conflicts**: Uses the same DDS library as Unitree SDK2
2. **Simpler Build**: No need for ROS2 environment or colcon
3. **Better Integration**: Native integration with Unitree's ecosystem
4. **Standalone**: Can run without ROS2 installation

## Troubleshooting

### Build Issues

If you encounter build errors, ensure:
1. Unitree SDK2 is properly installed
2. All dependencies are available
3. No conflicting ROS2 paths in CMAKE_PREFIX_PATH

### DDS Connection Issues

If reset commands are not received:
1. Check that both publisher and subscriber use the same domain ID
2. Verify network interface configuration
3. Ensure proper DDS initialization

### Example Commands

```bash
# Build
cd src/unitree_mujoco/simulate
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# Run simulation
./unitree_mujoco_dds -r g1 -n eno1 -i 0

# In another terminal, send reset
./dds_reset_publisher --once
``` 