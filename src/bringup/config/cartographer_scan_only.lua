-- Include configuration files "map_builder.lua" and "trajectory_builder.lua"
include "map_builder.lua"
include "trajectory_builder.lua"

-- ===== Pure scan matching mode (completely avoid arc-shaped paths during rotation) =====
-- Avoid ZED Wrapper odometry fluctuations,
-- Perform position estimation using only Cartographer scan matching
-- Maintain straight walls even during rotation without drawing arcs

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame  = "zed_camera_link",
  published_frame = "zed_odom",
  odom_frame = "zed_odom",
  provide_odom_frame = true,  -- Cartographer provides odom frame
  publish_frame_projected_to_2d = true,
  use_odometry = false,  -- Do not use external odometry
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
  odometry_sampling_ratio = 0.0,  -- Completely ignore odometry
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

-- ===== Online correlative scan matching (most critical) =====
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Real-time correlative scan matcher (wide range, fast search)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15  -- 0.2→0.15: Optimization
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(45.0)  -- 60→45 degrees: Optimization
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.1  -- Maximum search freedom
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1  -- Maximum search freedom

-- ===== Accumulated range data (key to avoiding arc-shaped paths) =====
-- Important: Accumulating multiple scans without odometry causes
-- inaccurate relative positions between accumulated scans, resulting in arcs
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 2→1: Immediate matching (arc prevention)

-- ===== Ceres scan matcher (high accuracy and speed balance) =====
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.0  -- 30→20: Balance adjustment
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 250.0  -- 300→250: Maintain high accuracy

-- Ceres optimization settings (accuracy and speed balance)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20  -- 100→20: Speed optimization
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4

-- ===== Motion filter (fine-grained updates) =====
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01  -- 1cm
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- 0.2→0.1 degrees: More fine-grained

-- ===== Submap configuration =====
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90  -- 60→90: Improved stability
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Occupancy probability update settings
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.70
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.30

-- ===== Pose graph optimization (100% scan matching dependency) =====
POSE_GRAPH.optimize_every_n_nodes = 90  -- 15→90: Suppress optimization (arc prevention)

-- Constraint builder settings
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 0.60→0.65: More strict
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70  -- 0.65→0.70
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- 0.5→0.3: Suppress sampling
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- Fast correlative scan matcher (for loop closure)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(45.0)  -- 60→45 degrees
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7  -- 8→7: Speed optimization

-- Ceres scan matcher (for loop closure)
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.0  -- 30→20
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 200.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20  -- 100→20: Speed optimization
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 4

-- ===== Optimization problem weight settings (100% scan matching) =====
POSE_GRAPH.optimization_problem.huber_scale = 1e3  -- 5e2→1e3: Stabilization

-- Set odometry weights to zero (complete ignore)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0

-- Maximize local SLAM (scan matching) weights
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e7
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e7

-- Acceleration constraint (suppress sudden changes)
POSE_GRAPH.optimization_problem.acceleration_weight = 1e4  -- 5e3→1e4: Smoother

-- Global optimization convergence settings
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50  -- 200→50: Speed optimization
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4

-- Optimization frequency and timing
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.max_num_final_iterations = 200  -- 500→200: Speed optimization

return options
