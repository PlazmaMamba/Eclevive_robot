-- Include configuration files "map_builder.lua" and "trajectory_builder.lua"
include "map_builder.lua"
include "trajectory_builder.lua"

-- Start of options configuration
options = {
  map_builder = MAP_BUILDER,  -- Map generation configuration object
  trajectory_builder = TRAJECTORY_BUILDER,  -- Trajectory generation configuration object
  map_frame = "map",  -- Map reference frame name
  --tracking_frame = "base_link",  -- Frame for tracking robot position
  --published_frame = "base_link",  -- Published frame name
  tracking_frame  = "zed_camera_link",  -- Frame for tracking robot position
  published_frame = "zed_odom",  -- Published frame name
  odom_frame = "zed_odom",  -- Odometry reference frame name
  --provide_odom_frame = false,  -- Automatically provide odometry frame
  provide_odom_frame = false, --true,  -- Automatically provide odometry frame
  --publish_frame_projected_to_2d = false,  -- Whether to project published frame to 2D
  publish_frame_projected_to_2d = true,  -- Whether to project published frame to 2D
  --use_odometry = false,  -- Whether to use odometry data
  use_odometry = true,  -- Whether to use odometry data
  use_nav_sat = false,  -- Whether to use GPS data
  use_landmarks = false,  -- Whether to use landmark data
  num_laser_scans = 1,  -- Number of laser scans to use
  num_multi_echo_laser_scans = 0,  -- Number of multi-echo laser scans
  num_subdivisions_per_laser_scan = 1,  -- Number of subdivisions per laser scan
  num_point_clouds = 0,  -- Number of point clouds to use
  lookup_transform_timeout_sec = 10.0,  -- Transform lookup timeout (seconds) - addresses time sync issues
  submap_publish_period_sec = 0.05,  -- Submap publishing period (seconds) - frequent updates even when stationary
  pose_publish_period_sec = 5e-3,  -- Pose publishing period (seconds)
  trajectory_publish_period_sec = 30e-3,  -- Trajectory publishing period (seconds)
  rangefinder_sampling_ratio = 1.0,  -- Rangefinder sampling ratio (use all data for free space updates)
  odometry_sampling_ratio = 1.,  -- Odometry sampling ratio (use all ZED odometry data)
  fixed_frame_pose_sampling_ratio = 1.,  -- Fixed frame pose sampling ratio
  imu_sampling_ratio = 1.,  -- IMU sampling ratio
  landmarks_sampling_ratio = 1.,  -- Landmarks sampling ratio
}

-- Configuration to use 2D trajectory generation
MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D trajectory generation configuration
TRAJECTORY_BUILDER_2D.min_range = 0.4  -- Minimum range for distance measurement (meters)
TRAJECTORY_BUILDER_2D.max_range = 12.0  -- Maximum range for distance measurement (meters) - prevent map expansion from noisy point clouds
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 12.0  -- Distance to use for missing data (set to max_range or higher)
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- Whether to use IMU data
--TRAJECTORY_BUILDER_2D.use_imu_data = true  -- Whether to use IMU data
--TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false  -- Whether to use online scan matching
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- Use online scan matching by motoms
-- Real-time scan matcher configuration (improved stability during rotation)
-- translation_delta_cost_weight: Cost weight for translation changes (higher value = suppress translation, reduce rotation jitter)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
-- rotation_delta_cost_weight: Cost weight for rotation changes (higher value = improve rotation estimation stability)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e3
-- Number of accumulated range data (scan data accumulation count)
-- Increasing value improves scan matching accuracy but increases computational load and response delay
-- 1: Fast but sensitive to noise, 2-3: Well-balanced, 4+: High accuracy but significant delay
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1                 -- 1 (improved stability when stationary) by motoms

-- POSE_GRAPH constraint builder score settings (loop closure reliability control)
-- min_score: Minimum confidence for local matching (0.0-1.0, higher value = stricter constraints, reduced false matching)
POSE_GRAPH.constraint_builder.min_score = 0.65  -- Minimum score for scan matching
-- global_localization_min_score: Minimum confidence for global localization (higher value = more reliable position estimation)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- Minimum score for global localization

-- POSE_GRAPH optimization problem weight settings (global map optimization)
-- local_slam_pose_translation_weight: Local SLAM translation confidence (higher value = prioritize local map)
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5  -- Weight for local SLAM translation
-- local_slam_pose_rotation_weight: Local SLAM rotation confidence (higher value = prioritize local rotation estimation)
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5  -- Weight for local SLAM rotation
-- odometry_translation_weight: Odometry translation confidence (higher value = prioritize wheel encoder)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5  -- Weight for odometry translation
-- odometry_rotation_weight: Odometry rotation confidence (higher value = prioritize wheel-based rotation estimation)
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5  -- Weight for odometry rotation
-- huber_scale: Robustness control for outliers (higher value = reduce outlier influence)
POSE_GRAPH.optimization_problem.huber_scale = 1e3  -- Huber loss function scale

-- Scan matching weight settings
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10  -- Weight for occupied space
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40  -- Weight for rotation
-- Ceres scan matcher weight settings (high-precision matching via nonlinear optimization)
-- occupied_space_weight: Weight for occupied space fit (higher value = improved obstacle detection accuracy, clearer map)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1  -- Weight for occupied space
-- rotation_weight: Weight for rotation estimation (higher value = improved angle estimation during rotation, reduced map distortion)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40  -- Weight for rotation

-- Submap configuration (local map segment management)
-- num_range_data: Number of scan data per submap (higher value = detailed map, lower value = lightweight processing)
-- Typical values: 60-120 (indoor), 200-400 (outdoor)
-- Reduce data count to increase update frequency for continued map updates when stationary
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60  -- Number of range data per submap (high-frequency updates when stationary)
-- Motion filter settings (filter out unnecessary small movements)
-- max_distance_meters: Ignore movements below this distance (small value = record fine movements, large value = record only large movements)
-- Relax filter value to enable map updates when stationary
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01  -- Maximum distance filter (meters) - map updates even when stationary
-- max_angle_radians: Ignore rotations below this angle (small value = record fine rotations, improved rotation accuracy)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.01)  -- Maximum angle filter (radians) - map updates even when stationary

--SPARSE_POSE_GRAPH.optimize_every_n_scans = 0 --by motoms Slipping occurs within submap, this is a local SLAM issue. Turn off global SLAM to avoid interfering with tuning.
POSE_GRAPH.optimize_every_n_nodes = 0

-- Additional settings to force map updates when stationary
-- Enable map probability value updates (continue map updates from scan data even when stationary)
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
-- Improve occupancy probability update frequency (enhanced obstacle detection accuracy when stationary)
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true
-- Enable free space insertion (promote updates of white areas)
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.7
-- Adjust hit probability (obstacle detection sensitivity)
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.3
-- Return final options configuration
return options

