# MuJoCo Simulation with ROS2 Reset Support

This is a modified version of the Unitree MuJoCo simulation that includes ROS2 support for robot reset functionality.

## Features

- **Original MuJoCo Simulation**: All original functionality is preserved
- **ROS2 Integration**: Minimal ROS2 node integration
- **Reset Command**: Subscribe to `/mjc/reset` topic to reset robot to default position
- **Thread-safe**: Reset commands are processed safely in the physics loop

## Building

### Prerequisites

- ROS2 (Foxy or later)
- MuJoCo library
- Unitree SDK2
- All original dependencies (glfw, yaml-cpp, boost, fmt)

### Build Instructions

```bash
# Navigate to the simulate directory
cd .ref/unitree_mujoco/simulate

# Build with ROS2 support
mkdir -p build && cd build
cmake .. -DBUILD_ROS2=ON
make -j$(nproc)

# Or if building as a ROS2 package
cd /path/to/your/ros2_workspace/src
# Copy or symlink the simulate directory here
colcon build --packages-select unitree_mujoco
```

## Usage

### 1. Start the ROS2 MuJoCo Simulation

```bash
# Method 1: Direct execution
cd .ref/unitree_mujoco/simulate/build
./unitree_mujoco_ros -r g1 -n eno1 -i 0

# Method 2: Using ROS2 launch (if built as ROS2 package)
ros2 launch unitree_mujoco mujoco_simulate.launch.py robot:=g1 interface:=eno1 domain_id:=0
```

### 2. Send Reset Commands

The simulator listens for reset commands on the `/mjc/reset` topic using `std_msgs/msg/Empty` messages.

#### Option A: Using ros2 command line
```bash
# Send a single reset command
ros2 topic pub --once /mjc/reset std_msgs/msg/Empty
```

#### Option B: Using the test script
```bash
# Run the test script (sends reset every 10 seconds)
python3 scripts/test_reset.py
```

#### Option C: Programmatic usage
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.reset_pub = self.create_publisher(Empty, '/mjc/reset', 10)
    
    def reset_robot(self):
        msg = Empty()
        self.reset_pub.publish(msg)
        self.get_logger().info('Reset command sent')
```

## Reset Functionality

When a reset command is received:

1. **Position Reset**: Robot joints return to default positions (keyframe 0 if available, otherwise `qpos0`)
2. **Velocity Reset**: All joint velocities are set to zero
3. **Force Reset**: All applied external forces are cleared
4. **Forward Kinematics**: `mj_forward()` is called to update derived quantities

## Topic Interface

| Topic        | Message Type         | Description                     |
| ------------ | -------------------- | ------------------------------- |
| `/mjc/reset` | `std_msgs/msg/Empty` | Reset robot to default position |

## Files Modified/Added

### Core Files
- `src/main_ros.cc` - ROS2 version of main.cc with reset functionality
- `CMakeLists.txt` - Updated to support ROS2 builds
- `package.xml` - ROS2 package definition

### Launch and Scripts
- `launch/mujoco_simulate.launch.py` - ROS2 launch file
- `scripts/test_reset.py` - Test script for reset functionality

### Key Changes to main_ros.cc

1. **Added ROS2 includes and node class**:
   ```cpp
   #include <rclcpp/rclcpp.hpp>
   #include <std_msgs/msg/Empty.hpp>
   
   class MuJoCoSimulateNode : public rclcpp::Node
   ```

2. **Added reset flag and function**:
   ```cpp
   std::atomic<bool> reset_requested{false};
   
   void ResetRobot() {
     // Reset implementation
   }
   ```

3. **Added reset check in physics loop**:
   ```cpp
   if (reset_requested.load()) {
     ResetRobot();
     reset_requested.store(false);
   }
   ```

4. **ROS2 integration in main()**:
   ```cpp
   // Initialize ROS2
   rclcpp::init(argc, argv);
   auto node = std::make_shared<MuJoCoSimulateNode>();
   
   // Start ROS2 spinner in separate thread
   std::thread ros_thread([node]() {
     rclcpp::spin(node);
   });
   ```

## Building Options

### Option 1: Standalone Build (Recommended for testing)
```bash
cd .ref/unitree_mujoco/simulate
mkdir build && cd build
cmake .. -DBUILD_ROS2=ON
make -j$(nproc)
```

### Option 2: ROS2 Package Build
```bash
# In your ROS2 workspace
colcon build --packages-select unitree_mujoco
source install/setup.bash
```

## Compatibility

- **Backward Compatible**: Original `unitree_mujoco` binary is still built
- **Minimal Changes**: Core simulation logic unchanged
- **Thread Safe**: Reset operations are atomic and safe
- **Performance**: Minimal overhead when no reset commands are sent

## Troubleshooting

1. **Build Errors**: Ensure all ROS2 dependencies are installed
2. **Topic Not Found**: Check that the ROS2 node started successfully
3. **Reset Not Working**: Verify the topic name is exactly `/mjc/reset`
4. **Permission Issues**: Make sure scripts are executable (`chmod +x scripts/test_reset.py`)

## Example Workflow

1. Start the simulation:
   ```bash
   ./unitree_mujoco_ros -r g1 -n eno1 -i 0
   ```

2. In another terminal, test the reset:
   ```bash
   ros2 topic pub --once /mjc/reset std_msgs/msg/Empty
   ```

3. Observe the robot returning to default position in the MuJoCo viewer

4. For continuous testing:
   ```bash
   python3 scripts/test_reset.py
   ``` 