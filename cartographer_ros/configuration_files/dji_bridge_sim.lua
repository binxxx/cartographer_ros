
-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = false,
  use_camera = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
}

TRAJECTORY_BUILDER_3D_ESKF.scans_per_accumulation = 10
TRAJECTORY_BUILDER_3D_ESKF.voxel_filter_size = 0.1
TRAJECTORY_BUILDER_3D_ESKF.submaps.high_resolution = 0.1
TRAJECTORY_BUILDER_3D_ESKF.submaps.low_resolution = 0.3
TRAJECTORY_BUILDER_3D_ESKF.submaps.num_range_data = 50
TRAJECTORY_BUILDER_3D_ESKF.submaps.initial_map_file_name = "/home/aeroscout/Documents/bridge_sim/bridge_sim.bt"
TRAJECTORY_BUILDER_3D_ESKF.submaps.initial_map_resolution = 0.2
TRAJECTORY_BUILDER_3D_ESKF.eskf.imu_frequency = 50.0


MAP_BUILDER.use_trajectory_builder_3d_eskf = true
MAP_BUILDER.num_background_threads = 7
MAP_BUILDER.sparse_pose_graph.optimization_problem.huber_scale = 5e3
MAP_BUILDER.sparse_pose_graph.optimize_every_n_scans = 20
MAP_BUILDER.sparse_pose_graph.constraint_builder.sampling_ratio = 0.3
MAP_BUILDER.sparse_pose_graph.optimization_problem.ceres_solver_options.max_num_iterations = 10
MAP_BUILDER.sparse_pose_graph.optimization_problem.rotation_weight = 3e1;
-- Reuse the coarser 3D voxel filter to speed up the computation of loop closure
-- constraints.
-- MAP_BUILDER.sparse_pose_graph.constraint_builder.adaptive_voxel_filter = TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter
MAP_BUILDER.sparse_pose_graph.constraint_builder.min_score = 0.62

return options
