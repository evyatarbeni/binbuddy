#!/bin/bash
###############################################################################
# Generate robot_description package
# Contains: URDF model, robot state publisher, TF tree
###############################################################################

set -e
WORKSPACE_SRC="${1:-$(pwd)}"
cd "$WORKSPACE_SRC"

echo "→ Generating robot_description..."

mkdir -p robot_description/{urdf,launch,meshes}

# CMakeLists.txt
cat > robot_description/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  launch
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
EOF

# package.xml
cat > robot_description/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>robot_description</name>
  <version>1.0.0</version>
  <description>URDF description for BinBuddy robot</description>
  <maintainer email="evyatarbeni@gmail.com">Evyatar Beni</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# URDF file
cat > robot_description/urdf/binbuddy.urdf.xacro << 'EOF'
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="binbuddy">

  <!-- Properties -->
  <xacro:property name="wheel_base" value="0.35"/>
  <xacro:property name="wheel_radius" value="0.075"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="base_length" value="0.5"/>
  <xacro:property name="base_width" value="0.4"/>
  <xacro:property name="base_height" value="0.15"/>
  <xacro:property name="crane_height" value="0.8"/>

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.2 0.2 0.8 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.1 0.1 0.1 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.5 0.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>

  <!-- Base footprint (on ground) -->
  <link name="base_footprint"/>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" 
               iyy="0.5" iyz="0.0" izz="0.5"/>
    </inertial>
  </link>

  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- LiDAR -->
  <link name="lidar_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.08"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" 
               iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.15 0 ${base_height/2 + 0.08}" rpy="0 0 0"/>
  </joint>

  <!-- USB Camera (front center) -->
  <link name="camera_usb_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.08 0.03"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <joint name="camera_usb_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_usb_link"/>
    <origin xyz="0.25 -0.08 ${base_height/2}" rpy="0 0 0"/>
  </joint>

  <!-- USB Camera optical frame -->
  <link name="camera_usb_optical"/>
  <joint name="camera_usb_optical_joint" type="fixed">
    <parent link="camera_usb_link"/>
    <child link="camera_usb_optical"/>
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
  </joint>

  <!-- Pi Camera 1 (front right) -->
  <link name="camera_picam1_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.08 0.03"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <joint name="camera_picam1_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_picam1_link"/>
    <origin xyz="0.25 0.08 ${base_height/2}" rpy="0 0 0"/>
  </joint>

  <link name="camera_picam1_optical"/>
  <joint name="camera_picam1_optical_joint" type="fixed">
    <parent link="camera_picam1_link"/>
    <child link="camera_picam1_optical"/>
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
  </joint>

  <!-- Pi Camera 2 (rear center) -->
  <link name="camera_picam2_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.08 0.03"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>

  <joint name="camera_picam2_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_picam2_link"/>
    <origin xyz="-0.25 0 ${base_height/2}" rpy="0 0 ${pi}"/>
  </joint>

  <link name="camera_picam2_optical"/>
  <joint name="camera_picam2_optical_joint" type="fixed">
    <parent link="camera_picam2_link"/>
    <child link="camera_picam2_optical"/>
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
  </joint>

  <!-- Crane base -->
  <link name="crane_base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" 
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="crane_base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="crane_base_link"/>
    <origin xyz="-0.15 0 ${base_height/2 + 0.1}" rpy="0 0 0"/>
  </joint>

  <!-- Crane arm (prismatic joint for vertical movement) -->
  <link name="crane_arm_link">
    <visual>
      <origin xyz="0 0 ${crane_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 ${crane_height}"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${crane_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 ${crane_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" 
               iyy="0.05" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="crane_arm_joint" type="prismatic">
    <parent link="crane_base_link"/>
    <child link="crane_arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0.0" upper="${crane_height}" effort="100.0" velocity="0.2"/>
  </joint>

  <!-- Crane gripper/hook -->
  <link name="crane_gripper_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
  </link>

  <joint name="crane_gripper_joint" type="fixed">
    <parent link="crane_arm_link"/>
    <child link="crane_gripper_link"/>
    <origin xyz="0 0 ${crane_height}" rpy="0 0 0"/>
  </joint>

</robot>
EOF

# Launch file for state publisher
cat > robot_description/launch/state_publisher.launch.py << 'EOFPYTHON'
#!/usr/bin/env python3
"""Launch robot state publisher with URDF"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'binbuddy.urdf.xacro'
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': False
        }]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher
    ])
EOFPYTHON

echo "  ✓ robot_description created"
