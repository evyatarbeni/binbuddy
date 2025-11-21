#!/bin/bash
###############################################################################
# Generate robot_interfaces package
# Contains: Custom messages, services, and actions for BinBuddy
###############################################################################

set -e
WORKSPACE_SRC="${1:-$(pwd)}"
cd "$WORKSPACE_SRC"

echo "→ Generating robot_interfaces..."

mkdir -p robot_interfaces/{msg,srv,action}

# CMakeLists.txt
cat > robot_interfaces/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CraneState.msg"
  "msg/SystemHealth.msg"
  "srv/SetMode.srv"
  "srv/CraneCommand.srv"
  "srv/NLPCommand.srv"
  "srv/CameraCommand.srv"
  "action/MoveCrane.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs action_msgs
)

ament_package()
EOF

# package.xml
cat > robot_interfaces/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>robot_interfaces</name>
  <version>1.0.0</version>
  <description>Custom interfaces for BinBuddy robot</description>
  <maintainer email="evyatarbeni@gmail.com">Evyatar Beni</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>action_msgs</depend>
  
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Messages
cat > robot_interfaces/msg/CraneState.msg << 'EOF'
# Crane position and status
float32 position_percent    # 0-100%
bool homed                  # Has crane been homed?
bool at_lower_limit         # Lower limit switch triggered
bool at_upper_limit         # Upper limit switch triggered
bool moving                 # Currently in motion
string current_preset       # "STOW", "PICK", "PLACE", "CUSTOM"
EOF

cat > robot_interfaces/msg/SystemHealth.msg << 'EOF'
# Overall system health status
bool arduino_connected
bool lidar_active
bool camera_usb_active
bool camera_picam1_active
bool camera_picam2_active
string active_camera        # "usb", "picam1", or "picam2"
bool movement_enabled       # Safety interlock
float32 battery_voltage     # Optional battery monitoring
EOF

# Services
cat > robot_interfaces/srv/SetMode.srv << 'EOF'
# Mode switching service
string mode  # "autonomy", "teleop", "estop"
---
bool success
string message
string previous_mode
EOF

cat > robot_interfaces/srv/CraneCommand.srv << 'EOF'
# Crane command service
string command              # "HOME", "STOW", "PICK", "PLACE", "GOTO"
float32 target_position     # 0-100 for GOTO command
---
bool success
string message
float32 final_position
EOF

cat > robot_interfaces/srv/NLPCommand.srv << 'EOF'
# Natural language command processing
string command_text
---
bool success
string interpretation
string[] planned_actions
string error_message
EOF

cat > robot_interfaces/srv/CameraCommand.srv << 'EOF'
# Camera control commands
string command              # "start_recording", "stop_recording", "snapshot"
string camera_name          # "usb", "picam1", "picam2"
---
bool success
string message
string file_path            # Path to saved file (if applicable)
EOF

# Actions
cat > robot_interfaces/action/MoveCrane.action << 'EOF'
# Crane movement action with feedback
float32 target_position     # Target position (0-100%)
float32 speed               # Movement speed (0-100%)
---
bool success
float32 final_position
---
float32 current_position    # Feedback during motion
EOF

echo "  ✓ robot_interfaces created"
