#!/usr/bin/env python3
"""
BinBuddy master bringup launch file
Launches all core nodes based on mode selection
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode', 
        default_value='teleop',
        description='Operating mode: teleop, autonomy, estop'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', 
        default_value='false',
        description='Start RViz visualization'
    )
    
    use_slam_arg = DeclareLaunchArgument(
        'use_slam', 
        default_value='false',
        description='Start SLAM (mapping mode)'
    )
    
    map_arg = DeclareLaunchArgument(
        'map', 
        default_value='',
        description='Path to map file for navigation'
    )
    
    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_slam = LaunchConfiguration('use_slam')
    
    # Robot description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch', 
                'state_publisher.launch.py'
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
            'movement_enabled': False,  # Must enable explicitly for safety
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
    
    # Teleop (always available, but only actively controlled when needed)
    teleop_node = Node(
        package='robot_teleop',
        executable='binbuddy_teleop',
        name='binbuddy_teleop',
        output='screen',
        prefix='xterm -e',  # Launch in separate window
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
            'rviz', 
            'binbuddy.rviz'
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
        nlp_node,
        rviz_node,
    ])
