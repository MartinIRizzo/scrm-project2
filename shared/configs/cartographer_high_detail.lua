-- High Detail Configuration
-- Use Case: Warehouse inventory mapping, precise obstacle detection
-- Trade-offs: Larger map files, higher CPU usage, slower real-time performance

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- VARIATION A: High Resolution (0.025m vs default 0.05m)
-- Captures finer details but increases computation and map file size
MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.025

-- VARIATION C: Aggressive scan matching (minimal filtering)
-- Keeps more laser points for detailed mapping
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 12.0  -- Extended range for larger spaces
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.01  -- Minimal point filtering
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.

-- VARIATION D: Trust sensors more than odometry (frequent updates)
-- Updates map more often based on laser scans
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1  -- Update every 10cm
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(5)  -- Update every 5 degrees

-- Loop closure settings
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5

return options
