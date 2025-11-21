#!/usr/bin/env bash
###############################################################################
# Generate robot_bringup package
# Contains: Master launch files for all modes
###############################################################################

set -e
WORKSPACE_SRC="${1:-$(pwd)}"
cd "$WORKSPACE_SRC"

echo "→ Generating robot_bringup..."

mkdir -p robot_bringup/{launch,config,rviz,robot_bringup,resource}
touch robot_bringup/resource/robot_bringup
touch robot_bringup/robot_bringup/__init__.py

# package.xml
cat > robot_bringup/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>robot_bringup</name>
  <version>1.0.0</version>
  <description>Master launch files for BinBuddy</description>
  <maintainer email="evyatarbeni@gmail.com">Evyatar Beni</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_interfaces</exec_depend>
  <exec_depend>robot_base</exec_depend>
  <exec_depend>crane_control</exec_depend>
  <exec_depend>robot_description</exec_depend>
  <exec_depend>robot_sensing</exec_depend>
  <exec_depend>robot_teleop</exec_depend>
  <exec_depend>robot_nlp</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
EOF

# CMakeLists.txt
cat > robot_bringup/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(robot_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
ament_package()
EOF

# Main bringup launch file
cat > robot_bringup/launch/bringup.launch.py << 'EOFPYTHON'
#!/usr/bin/env python3
"""
BinBuddy master bringup launch file
Launches all core nodes based on mode selection
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='teleop',
        description='Operating mode: teleop, autonomy, estop')
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Start RViz visualization')
    
    use_slam_arg = DeclareLaunchArgument(
        'use_slam', default_value='false',
        description='Start SLAM (mapping mode)')
    
    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to map file for navigation')
    
    # Get launch configurations
    mode = LaunchConfiguration('mode')
    use_rviz = LaunchConfiguration('use_rviz')
    use_slam = LaunchConfiguration('use_slam')
    map_file = LaunchConfiguration('map')
    
    # Robot description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch', 'state_publisher.launch.py'
            ])
        ])
    )
    
    # Base controller (motors + odometry)
    base_controller_node = Node(
        package='robot_base',
        executable='base_controller',
        name='base_controller',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'movement_enabled': False,  # Must enable explicitly
        }]
    )
    
    # Crane controller
    crane_controller_node = Node(
        package='crane_control',
        executable='crane_controller',
        name='crane_controller',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'movement_enabled': False,
        }]
    )
    
    # LiDAR driver
    lidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar_publisher',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'lidar_link',
            'laser_scan_topic_name': 'scan',
        }]
    )
    
    # Camera manager
    camera_manager_node = Node(
        package='robot_sensing',
        executable='camera_manager',
        name='camera_manager',
        output='screen',
        parameters=[{
            'default_camera': 'usb',
            'frame_rate': 15,
        }]
    )
    
    # Teleop (conditional on mode=teleop)
    teleop_node = Node(
        package='robot_teleop',
        executable='binbuddy_teleop',
        name='binbuddy_teleop',
        output='screen',
        condition=IfCondition(LaunchConfiguration('mode', default='teleop'))
    )
    
    # NLP commander
    nlp_node = Node(
        package='robot_nlp',
        executable='nlp_commander',
        name='nlp_commander',
        output='screen'
    )
    
    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('robot_bringup'),
            'rviz', 'binbuddy.rviz'
        ])],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        mode_arg,
        use_rviz_arg,
        use_slam_arg,
        map_arg,
        robot_description_launch,
        base_controller_node,
        crane_controller_node,
        lidar_node,
        camera_manager_node,
        teleop_node,
        nlp_node,
        rviz_node,
    ])
EOFPYTHON

# RViz configuration
cat > robot_bringup/rviz/binbuddy.rviz << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Displays:
    - Class: rviz_common/Grid
      Name: Grid
      Enabled: true
    - Class: rviz_common/RobotModel
      Name: RobotModel
      Enabled: true
      Topic: robot_description
    - Class: rviz_common/LaserScan
      Name: LaserScan
      Enabled: true
      Topic: /scan
    - Class: rviz_common/Camera
      Name: Camera
      Enabled: true
      Topic: /camera/image_raw
    - Class: rviz_common/TF
      Name: TF
      Enabled: true
    - Class: nav2_rviz_plugins/Map
      Name: Map
      Enabled: true
      Topic: /map
  Views:
    Current:
      Class: rviz_common/Orbit
      Distance: 5.0
EOF

echo "  ✓ robot_bringup created"
