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
  published_frame = "odom",  -- Published frame name
  odom_frame = "odom",  -- Odometry reference frame name
  provide_odom_frame = false,  -- Automatically provide odometry frame
  --provide_odom_frame = true,  -- Automatically provide odometry frame
  publish_frame_projected_to_2d = false,  -- Whether to project published frame to 2D
  --publish_frame_projected_to_2d = true,  -- Whether to project published frame to 2D
  use_odometry = false,  -- Whether to use odometry data
  --use_odometry = ture,  -- Whether to use odometry data
  use_nav_sat = false,  -- Whether to use GPS data
  use_landmarks = false,  -- Whether to use landmark data
  num_laser_scans = 1,  -- Number of laser scans to use
  num_multi_echo_laser_scans = 0,  -- Number of multi-echo laser scans
  num_subdivisions_per_laser_scan = 1,  -- Number of subdivisions per laser scan
  num_point_clouds = 0,  -- Number of point clouds to use
  lookup_transform_timeout_sec = 0.2,  -- Transform lookup timeout (seconds)
  submap_publish_period_sec = 0.1,  -- Submap publishing period (seconds)
  pose_publish_period_sec = 5e-3,  -- Pose publishing period (seconds)
  trajectory_publish_period_sec = 20e-3,  -- Trajectory publishing period (seconds)
  rangefinder_sampling_ratio = 1.,  -- Rangefinder sampling ratio
  odometry_sampling_ratio = 1.,  -- Odometry sampling ratio
  fixed_frame_pose_sampling_ratio = 1.,  -- Fixed frame pose sampling ratio
  imu_sampling_ratio = 1.,  -- IMU sampling ratio
  landmarks_sampling_ratio = 1.,  -- Landmarks sampling ratio
}

-- Configuration to use 2D trajectory generation
MAP_BUILDER.use_trajectory_builder_2d = true
--MAP_BUILDER.use_trajectory_builder_2d = false

-- 2D trajectory generation configuration
--TRAJECTORY_BUILDER_2D.min_range = 0.4  -- Minimum range for distance measurement (meters)
TRAJECTORY_BUILDER_2D.min_range = 0.3  -- Minimum range for distance measurement (meters)
TRAJECTORY_BUILDER_2D.max_range = 25.0  -- Maximum range for distance measurement (meters)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 25.5  -- Distance to use for missing data (meters)
--TRAJECTORY_BUILDER_2D.use_imu_data = false  -- Whether to use IMU data
TRAJECTORY_BUILDER_2D.use_imu_data = true  -- Whether to use IMU data
--TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false  -- Whether to use online scan matching
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- Use online scan matching by motoms
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1                 -- 1 (distortion countermeasure. Kept at 1 or 2 to avoid response degradation) by motoms

-- POSE_GRAPH constraint builder score settings
POSE_GRAPH.constraint_builder.min_score = 0.65  -- Minimum score for scan matching
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- Minimum score for global localization

-- POSE_GRAPH optimization problem weight settings
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5  -- Weight for local SLAM translation
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5  -- Weight for local SLAM rotation
POSE_GRAPH.optimization_problem.odometry_translation_weight = 7e4 --1e5  -- Weight for odometry translation
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5  -- Weight for odometry rotation
POSE_GRAPH.optimization_problem.huber_scale = 1e3  -- Huber loss function scale

-- Scan matching weight settings
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10  -- Weight for occupied space
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40  -- Weight for rotation
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 5  -- Weight for occupied space
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40  -- Weight for rotation
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20  -- Weight for rotation

-- Submap and motion filter settings
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120  -- Number of range data per submap
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1  -- Maximum distance filter (meters)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.18) --math.rad(0.12) --math.rad(0.08)  -- Maximum angle filter (radians)

--SPARSE_POSE_GRAPH.optimize_every_n_scans = 0 --by motoms Slipping occurs within submap, this is a local SLAM issue. Turn off global SLAM to avoid interfering with tuning.
POSE_GRAPH.optimize_every_n_nodes = 0
-- Return final options configuration
return options

