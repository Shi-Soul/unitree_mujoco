cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

# Check if we're building with ROS2
option(BUILD_ROS2 "Build ROS2 version" OFF)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(mujoco REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

# ROS2 dependencies (only if building ROS2 version)
if(BUILD_ROS2)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(ament_cmake REQUIRED)
endif()

file(GLOB SIM_SRC
    src/joystick/joystick.cc
    src/mujoco/*.cc
)

set(SIM_DEPENDENCIES
    pthread
    mujoco::mujoco
    glfw
    yaml-cpp
    unitree_sdk2
    boost_program_options
    fmt
)

# Build original version
add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco ${SIM_DEPENDENCIES})

# Build ROS2 version if requested
if(BUILD_ROS2)
  set(ROS2_DEPENDENCIES
      ${SIM_DEPENDENCIES}
      rclcpp
      std_msgs
  )
  
  add_executable(unitree_mujoco_ros ${SIM_SRC} src/main_ros.cc)
  target_link_libraries(unitree_mujoco_ros ${ROS2_DEPENDENCIES})
  
  # ROS2 specific settings
  ament_target_dependencies(unitree_mujoco_ros
    rclcpp
    std_msgs
  )
  
  install(TARGETS unitree_mujoco_ros
    DESTINATION lib/${PROJECT_NAME}
  )
  
  # Install launch files
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
  
  # Install scripts
  install(PROGRAMS scripts/test_reset.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)

# Install original executable
install(TARGETS unitree_mujoco
  DESTINATION bin
)

# Install ROS2 package if building ROS2 version
if(BUILD_ROS2)
  ament_package()
endif()