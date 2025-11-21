#!/bin/bash
###############################################################################
# Generate robot_navigation and robot_slam packages
# Contains: Nav2 and SLAM Toolbox configuration files
###############################################################################

set -e
WORKSPACE_SRC="${1:-$(pwd)}"
cd "$WORKSPACE_SRC"

echo "→ Generating robot_navigation and robot_slam..."

###############################################################################
# ROBOT_NAVIGATION
###############################################################################
mkdir -p robot_navigation/{config,maps,behavior_trees}

# CMakeLists.txt
cat > robot_navigation/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(robot_navigation)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config maps behavior_trees DESTINATION share/${PROJECT_NAME})
ament_package()
EOF

# package.xml
cat > robot_navigation/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>robot_navigation</name>
  <version>1.0.0</version>
  <description>Navigation configuration for BinBuddy</description>
  <maintainer email="evyatarbeni@gmail.com">Evyatar Beni</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>navigation2</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
EOF

# Nav2 parameters
cat > robot_navigation/config/nav2_params.yaml << 'EOF'
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 10.0
    laser_min_range: 0.15
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "differential"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.25
      stateful: True
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.4
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.4
      min_speed_theta: 0.0
      acc_lim_x: 0.5
      acc_lim_y: 0.0
      acc_lim_theta: 1.5
      decel_lim_x: -0.5
      decel_lim_y: 0.0
      decel_lim_theta: -1.5
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 40
      sim_time: 2.0
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

planner_server:
  ros__parameters:
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

behavior_server:
  ros__parameters:
    use_sim_time: False
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    wait:
      plugin: "nav2_behaviors::Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 1.5

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: False
      global_frame: odom
      robot_base_frame: base_link
      update_frequency: 5.0
      publish_frequency: 2.0
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      footprint: "[ [0.25, 0.2], [0.25, -0.2], [-0.25, -0.2], [-0.25, 0.2] ]"
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.4
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: False
      robot_base_frame: base_link
      global_frame: map
      update_frequency: 1.0
      publish_frequency: 1.0
      footprint: "[ [0.25, 0.2], [0.25, -0.2], [-0.25, -0.2], [-0.25, 0.2] ]"
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5
      always_send_full_costmap: True
EOF

# Example map (empty - user creates real maps)
cat > robot_navigation/maps/example_map.yaml << 'EOF'
image: example_map.pgm
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
EOF

# Create placeholder PGM
echo "P5 200 200 255" > robot_navigation/maps/example_map.pgm
dd if=/dev/zero bs=1 count=40000 2>/dev/null >> robot_navigation/maps/example_map.pgm

###############################################################################
# ROBOT_SLAM
###############################################################################
mkdir -p robot_slam/config

# CMakeLists.txt
cat > robot_slam/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(robot_slam)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
EOF

# package.xml
cat > robot_slam/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>robot_slam</name>
  <version>1.0.0</version>
  <description>SLAM configuration for BinBuddy</description>
  <maintainer email="evyatarbeni@gmail.com">Evyatar Beni</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
EOF

# SLAM Toolbox configuration
cat > robot_slam/config/slam_toolbox_params.yaml << 'EOF'
slam_toolbox:
  ros__parameters:
    use_sim_time: False
    
    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    mode: mapping  # or localization

    # Map management
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 10.0
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.5
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1  
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true 
    loop_match_minimum_chain_size: 10           
    loop_match_maximum_variance_coarse: 3.0  
    loop_match_minimum_response_coarse: 0.35    
    loop_match_minimum_response_fine: 0.45

    # Correlation parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1 

    # Loop closure params
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan matcher params
    distance_variance_penalty: 0.5      
    angle_variance_penalty: 1.0    

    fine_search_angle_offset: 0.00349     
    coarse_search_angle_offset: 0.349   
    coarse_angle_resolution: 0.0349        
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
EOF

echo "  ✓ robot_navigation and robot_slam created"
