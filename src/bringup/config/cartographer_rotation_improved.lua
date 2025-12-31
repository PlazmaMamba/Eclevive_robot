-- Include configuration files "map_builder.lua" and "trajectory_builder.lua"
include "map_builder.lua"
include "trajectory_builder.lua"

-- Start of options configuration
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame  = "zed_camera_link",
  published_frame = "zed_odom",
  odom_frame = "zed_odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 10.0,
  submap_publish_period_sec = 0.05,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,  -- Use odometry (but with lower weight)
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ===== Critical settings to prevent arc-shaped paths during rotation =====

-- Scan range configuration
TRAJECTORY_BUILDER_2D.min_range = 0.4
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- ===== Online correlative scan matching (rotation correction as top priority) =====
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Real-time correlative scan matcher
-- Significantly relax cost to prioritize scan matching
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(45.0)  -- ±45 degrees
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.0  -- Reduce odometry dependency
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0  -- Reduce odometry dependency

-- Accumulated range data (improves stability during rotation)
-- Accumulate multiple scans to improve matching accuracy
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 3  -- 1→3: Significantly improves stability during rotation

-- ===== Ceres scan matcher (prioritize scan matching with nonlinear optimization) =====
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0  -- 10→20: Emphasize scan matching
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 200.0  -- 100→200: Rotation correction as top priority

-- Ceres optimization settings (high accuracy and speed)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50  -- 30→50: Improved convergence during rotation
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4

-- ===== Motion filter =====
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0  -- 0.2→5.0: Relax timeout
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01  -- 0.02→0.01: Update more frequently
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)  -- 0.5→0.2 degrees: Update more frequently

-- ===== Submap configuration =====
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60  -- 80→60: Increase update frequency
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Occupancy probability update settings
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.70
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.30

-- ===== Pose graph optimization (reduce odometry dependency) =====
POSE_GRAPH.optimize_every_n_nodes = 20  -- 30→20: Optimize more frequently

-- Constraint builder settings
POSE_GRAPH.constraint_builder.min_score = 0.55  -- 0.60→0.55: Relax constraints slightly
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60  -- 0.65→0.60
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- Fast correlative scan matcher (for loop closure)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.0)  -- 30→45 degrees
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 8

-- Ceres scan matcher (for loop closure)
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 100.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 4

-- ===== Optimization problem weight settings (scan matching as top priority) =====
POSE_GRAPH.optimization_problem.huber_scale = 1e3
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3

-- Significantly reduce odometry weights (fundamental countermeasure for arc-shaped paths)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3  -- 1e5→1e3: Significantly reduce odometry dependency
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3  -- 1e5→1e3: Reduce rotation odometry dependency

-- Prioritize local SLAM (scan matching) weights
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e6  -- 1e5→1e6: Scan matching as top priority
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e6  -- 1e5→1e6: Rotation scan matching as top priority

-- Global optimization convergence settings
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4

-- Optimization frequency and timing
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.max_num_final_iterations = 400

return options
