/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "glog/logging.h"
#include "octomap_msgs/conversions.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {

namespace {

cartographer_ros_msgs::SensorTopics DefaultSensorTopics() {
  cartographer_ros_msgs::SensorTopics topics;
  topics.laser_scan_topic = kLaserScanTopic;
  topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
  topics.point_cloud2_topic = kPointCloud2Topic;
  topics.imu_topic = kImuTopic;
  topics.odometry_topic = kOdometryTopic;
  topics.image_topic = kImageTopic;
  return topics;
}

// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    void (Node::*handler)(int, const string&,
                          const typename MessageType::ConstPtr&),
    const int trajectory_id, const string& topic,
    ::ros::NodeHandle* const node_handle, Node* const node) {
  return node_handle->subscribe<MessageType>(
      topic, kInfiniteSubscriberQueueSize,
      boost::function<void(const typename MessageType::ConstPtr&)>(
          [node, handler, trajectory_id,
           topic](const typename MessageType::ConstPtr& msg) {
            (node->*handler)(trajectory_id, topic, msg);
          }));
}

}  // namespace

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

Node::Node(const NodeOptions& node_options, tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, tf_buffer) {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  octomap_publisher_ = 
      node_handle_.advertise<::octomap_msgs::Octomap>(
          kOctomapTopic, kLatestOnlyPublisherQueueSize);
  pose_publisher_ = node_handle_.advertise<::geometry_msgs::PoseStamped>(
          kPoseTopic, kLatestOnlyPublisherQueueSize);
  boundingbox_publisher_ = node_handle_.advertise<::visualization_msgs::Marker>(
          kBoundingBoxTopic, kLatestOnlyPublisherQueueSize);
  map_point_cloud_publisher_ = 
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kMapPointCloudTopic, kLatestOnlyPublisherQueueSize);
  path_publisher_ =
          node_handle_.advertise<nav_msgs::Path>(
          kPathTopic, kLatestOnlyPublisherQueueSize);
  service_servers_.push_back(node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteStateServiceName, &Node::HandleWriteState, this));

  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.pose_publish_period_sec),
      &Node::PublishTrajectoryStates, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishTrajectoryNodeList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kConstraintPublishPeriodSec),
      &Node::PublishConstraintList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(2.0),
      &Node::PublishOctomap, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(0.1),
      &Node::PublishSubmapTf, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(0.1),
      &Node::PublishPose, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(1.0),
      &Node::PublishMapPointCloud, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(2.0),
      &Node::PublishPath, this));
  prev_stamp = 0.0;
}

Node::~Node() {}

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  return map_builder_bridge_.HandleSubmapQuery(request, response);
}

void Node::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
}
void Node::PublishSubmapTf(const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);    
  if (::ros::Time::now().toSec() <=0) return;
  // Publish submap frame tf
  geometry_msgs::TransformStamped stamped_transform;
  stamped_transform.header.stamp = ::ros::Time::now();
  stamped_transform.transform = ToGeometryMsgTransform(
        map_builder_bridge_.GetMatchingLocalPose());
  stamped_transform.header.frame_id = node_options_.map_frame;
  stamped_transform.child_frame_id = "submap";
  tf_broadcaster_.sendTransform(stamped_transform);

  geometry_msgs::TransformStamped map_to_map_view;
  map_to_map_view.header.stamp = ::ros::Time::now();
  map_to_map_view.transform = ToGeometryMsgTransform(
      carto::transform::Rigid3d(
        Eigen::Vector3d(0.0,0.0,0.0),
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())));
  map_to_map_view.header.frame_id = node_options_.map_frame;
  map_to_map_view.child_frame_id = "map_view";
  tf_broadcaster_.sendTransform(map_to_map_view);

}

void Node::PublishPose(const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);    
  if (::ros::Time::now().toSec() <=0) return;
  // Publish submap frame tf
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ::ros::Time::now();

  carto::transform::Rigid3d latest_pose = map_builder_bridge_.pose_estimate().pose;
  msg.pose.position.x = latest_pose.translation().x();
  msg.pose.position.y = latest_pose.translation().y();
  msg.pose.position.z = latest_pose.translation().z();
  msg.pose.orientation.w = latest_pose.rotation().w();
  msg.pose.orientation.x = latest_pose.rotation().x();
  msg.pose.orientation.y = latest_pose.rotation().y();
  msg.pose.orientation.z = latest_pose.rotation().z();

  pose_publisher_.publish(msg);
}


void Node::PublishOctomap(const ::ros::WallTimerEvent& unused_timer_event) {
    carto::common::MutexLocker lock(&mutex_);    
    if (::ros::Time::now().toSec() <=0) return;

    // std::cout << "hahahahah \n";
    matching_octomap_ = map_builder_bridge_.GetMatchingOctomap();  
    // std::cout << "haha" << std::endl;
    if (matching_octomap_ != nullptr && 
        map_builder_bridge_.GetMatchingSubmap() != nullptr) {
      octomap_msgs::Octomap octomap_msg;
      octomap_msg.header.stamp = ::ros::Time::now()-::ros::Duration(0.1);
      octomap_msg.header.frame_id = "submap";
      octomap_msgs::binaryMapToMsg(*matching_octomap_, octomap_msg);
      octomap_publisher_.publish(octomap_msg);
    
      // bounding box of 
      ::visualization_msgs::Marker line_lists;
      line_lists.header.frame_id = "submap";
      line_lists.header.stamp = ::ros::Time::now();
      line_lists.ns = "points_and_lines";
      line_lists.action = visualization_msgs::Marker::ADD;
      line_lists.pose.orientation.w = 1.0;
      line_lists.id = 0;
      line_lists.type = visualization_msgs::Marker::LINE_LIST;
      line_lists.scale.x = 0.1;
      line_lists.color.r = 1.0;
      line_lists.color.a = 1.0;

      double min_x, min_y, min_z;
      double max_x, max_y, max_z;
      std::vector<double> bounding_box = 
          map_builder_bridge_.GetMatchingSubmap()
              ->GetDistanceMapBoundingBox();

      min_x = bounding_box[0];
      min_y = bounding_box[1];
      min_z = bounding_box[2];
      max_x = bounding_box[3];
      max_y = bounding_box[4];
      max_z = bounding_box[5];
      geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
      p1.x = min_x; p1.y = min_y; p1.z = min_z;
      p2.x = max_x; p2.y = min_y; p2.z = min_z;
      p3.x = max_x; p3.y = max_y; p3.z = min_z;
      p4.x = min_x; p4.y = max_y; p4.z = min_z;
      p5.x = min_x; p5.y = min_y; p5.z = max_z;
      p6.x = max_x; p6.y = min_y; p6.z = max_z;
      p7.x = max_x; p7.y = max_y; p7.z = max_z;
      p8.x = min_x; p8.y = max_y; p8.z = max_z;

      line_lists.points.push_back(p1);
      line_lists.points.push_back(p2);
      line_lists.points.push_back(p1);
      line_lists.points.push_back(p4);
      line_lists.points.push_back(p1);
      line_lists.points.push_back(p5);

      line_lists.points.push_back(p2);
      line_lists.points.push_back(p3);
      line_lists.points.push_back(p2);
      line_lists.points.push_back(p6);

      line_lists.points.push_back(p3);
      line_lists.points.push_back(p7);
      line_lists.points.push_back(p3);
      line_lists.points.push_back(p4);

      line_lists.points.push_back(p4);
      line_lists.points.push_back(p8);
      line_lists.points.push_back(p5);
      line_lists.points.push_back(p6);
      line_lists.points.push_back(p5);
      line_lists.points.push_back(p8);

      line_lists.points.push_back(p6);
      line_lists.points.push_back(p7);
      line_lists.points.push_back(p7);
      line_lists.points.push_back(p8);


      boundingbox_publisher_.publish(line_lists);
    }
    matching_octomap_.reset();

}

void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

void Node::PublishMapPointCloud(const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  
  carto::sensor::PointCloud map_cloud;
  std::vector<std::vector<carto::mapping::TrajectoryNode>> 
  trajectory_nodes = map_builder_bridge_.map_builder()->
                                         sparse_pose_graph()->
                                         GetTrajectoryNodes();
  carto::common::Time last_state_time;
  for (size_t trajectory_id = 0; trajectory_id < trajectory_nodes.size(); 
        ++trajectory_id ) {

    // size_t node_stride = 1+size_t(floor(double(trajectory_nodes.at(trajectory_id).size()) / 10.0));
    size_t node_stride = 1;
    for (size_t node_index = 0; node_index < trajectory_nodes.at(trajectory_id).size(); 
          node_index+=node_stride) {
        // std::cout << "trajectory node size: " << trajectory_nodes.at(trajectory_id).size() << "!!!!!!!!!!!!" << std::endl;
        carto::mapping::TrajectoryNode trajectory_node = 
          trajectory_nodes.at(trajectory_id).at(node_index);
        
        // carto::sensor::RangeData scan_range_data = 
        //   carto::sensor::Decompress(trajectory_node.constant_data->high_resolution_point_cloud);
        
        carto::sensor::PointCloud scan_cloud = 
          carto::sensor::TransformPointCloud(
              trajectory_node.constant_data
                ->high_resolution_point_cloud, 
              trajectory_node.pose.cast<float>());
        
        map_cloud.insert(map_cloud.end(), 
                         scan_cloud.begin(), 
                         scan_cloud.end());
        last_state_time = trajectory_node.time();     
    }
  }
  if(carto::common::ToUniversal(last_state_time) > 0) {
    // map_point_cloud_publisher_.publish(ToPointCloud2Message(
    //       carto::common::ToUniversal(last_state_time),
    //       "map",
    //       carto::sensor::VoxelFiltered(map_cloud, 0.1)));
    map_point_cloud_publisher_.publish(ToPointCloud2Message(
          carto::common::ToUniversal(last_state_time),
          "map",
          map_cloud));
  }
}

void Node::PublishPath(const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  // nav_msgs::Path path_msg;

  std::vector<std::vector<carto::mapping::TrajectoryNode>> 
  trajectory_nodes = map_builder_bridge_.map_builder()->
                                         sparse_pose_graph()->
                                         GetTrajectoryNodes();
  
  std::ofstream file;
  file.open("/home/aeroscout/Documents/highbay_carto/aria_single_2_path.csv");
  ::ros::Time pose_time;
  for (size_t trajectory_id = 0; trajectory_id < trajectory_nodes.size(); 
        ++trajectory_id ) {
    for (size_t node_index = 0; node_index < trajectory_nodes.at(trajectory_id).size(); 
          ++node_index) {
        carto::mapping::TrajectoryNode trajectory_node = 
          trajectory_nodes.at(trajectory_id).at(node_index);

        carto::transform::Rigid3d pose = trajectory_node.pose;
        pose_time = ToRos(trajectory_node.time());
   
        // geometry_msgs::PoseStamped pose_msg;
        // pose_msg.header.frame_id = "map";
        // pose_msg.header.stamp = ToRos(pose_time);
        // pose_msg.pose.position.x = pose.translation().x();
        // pose_msg.pose.position.y = pose.translation().y();
        // pose_msg.pose.position.z = pose.translation().z();

        // pose_msg.pose.orientation.w = pose.rotation().w();
        // pose_msg.pose.orientation.x = pose.rotation().x();
        // pose_msg.pose.orientation.y = pose.rotation().y();
        // pose_msg.pose.orientation.z = pose.rotation().z();

        // path_msg.poses.push_back(pose_msg);
        file << pose_time.toNSec() << ", " 
             << pose.translation().x() << ", "
             << pose.translation().y() << ", "
             << pose.translation().z() << ", "
             << pose.rotation().x() << ", "
             << pose.rotation().y() << ", "
             << pose.rotation().z() << ", "
             << pose.rotation().w() << ", \n";
    }
  }
  file.close();
  // path_publisher_.publish(path_msg);
}
void Node::PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    const auto& trajectory_state = entry.second;

    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_state.pose_estimate.time != extrapolator.GetLastPoseTime()) {
      scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
          carto::common::ToUniversal(trajectory_state.pose_estimate.time),
          node_options_.map_frame,
          carto::sensor::TransformPointCloud(
              trajectory_state.pose_estimate.point_cloud,
              trajectory_state.local_to_map.cast<float>())));
      extrapolator.AddPose(trajectory_state.pose_estimate.time,
                           trajectory_state.pose_estimate.pose);
    }

    geometry_msgs::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now =
        std::max(FromRos(ros::Time::now()), extrapolator.GetLastPoseTime());
    stamped_transform.header.stamp = ToRos(now);
    const Rigid3d tracking_to_local = extrapolator.ExtrapolatePose(now);
    const Rigid3d tracking_to_map =
        trajectory_state.local_to_map * tracking_to_local;

    if (trajectory_state.published_to_tracking != nullptr) {
      if (trajectory_state.trajectory_options.provide_odom_frame) {
        std::vector<geometry_msgs::TransformStamped> stamped_transforms;

        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.transform =
            ToGeometryMsgTransform(trajectory_state.local_to_map);
        stamped_transforms.push_back(stamped_transform);

        stamped_transform.header.frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_local * (*trajectory_state.published_to_tracking));
        stamped_transforms.push_back(stamped_transform);

        tf_broadcaster_.sendTransform(stamped_transforms);
      } else {
        // std::cout << "Node::PublishTrajectoryStates::Publishing TF." << std::endl;
        // stamped_transform.header.frame_id = node_options_.map_frame;
        // stamped_transform.child_frame_id =
        //     trajectory_state.trajectory_options.published_frame;
        // stamped_transform.transform = ToGeometryMsgTransform(
        //     tracking_to_map * (*trajectory_state.published_to_tracking));
        const Rigid3d map_to_published_frame = 
            tracking_to_map * (*trajectory_state.published_to_tracking);
        stamped_transform.header.frame_id = 
            trajectory_state.trajectory_options.published_frame;
        stamped_transform.child_frame_id = node_options_.map_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            map_to_published_frame.inverse());

        tf_broadcaster_.sendTransform(stamped_transform);
      }
    }
  }
}

void Node::PublishTrajectoryNodeList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  if (trajectory_node_list_publisher_.getNumSubscribers() > 0) {
    trajectory_node_list_publisher_.publish(
        map_builder_bridge_.GetTrajectoryNodeList());
  }
}

void Node::PublishConstraintList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  if (constraint_list_publisher_.getNumSubscribers() > 0) {
    constraint_list_publisher_.publish(map_builder_bridge_.GetConstraintList());
  }
}

std::unordered_set<string> Node::ComputeExpectedTopics(
    const TrajectoryOptions& options,
    const cartographer_ros_msgs::SensorTopics& topics) {
  std::unordered_set<string> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    expected_topics.insert(topic);
  }
  for (const string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    expected_topics.insert(topic);
  }
  for (const string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    expected_topics.insert(topic);
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d_eskf() ||
      node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(topics.imu_topic);
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(topics.odometry_topic);
  }
  return expected_topics;
}

int Node::AddTrajectory(const TrajectoryOptions& options,
                        const cartographer_ros_msgs::SensorTopics& topics) {
  const std::unordered_set<string> expected_sensor_ids =
      ComputeExpectedTopics(options, topics);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  LaunchSubscribers(options, topics, trajectory_id);
  is_active_trajectory_[trajectory_id] = true;
  subscribed_topics_.insert(expected_sensor_ids.begin(),
                            expected_sensor_ids.end());
  return trajectory_id;
}

void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const cartographer_ros_msgs::SensorTopics& topics,
                             const int trajectory_id) {
  for (const string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        SubscribeWithHandler<sensor_msgs::LaserScan>(
            &Node::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,
            this));
  }
  for (const string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        SubscribeWithHandler<sensor_msgs::MultiEchoLaserScan>(
            &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
            &node_handle_, this));
  }
  for (const string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        SubscribeWithHandler<sensor_msgs::PointCloud2>(
            &Node::HandlePointCloud2Message, trajectory_id, topic,
            &node_handle_, this));
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d_eskf() ||
      node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    string topic = topics.imu_topic;
    subscribers_[trajectory_id].push_back(
        SubscribeWithHandler<sensor_msgs::Imu>(&Node::HandleImuMessage,
                                               trajectory_id, topic,
                                               &node_handle_, this));
  }

  if (options.use_odometry) {
    string topic = topics.odometry_topic;
    subscribers_[trajectory_id].push_back(
        SubscribeWithHandler<nav_msgs::Odometry>(&Node::HandleOdometryMessage,
                                                 trajectory_id, topic,
                                                 &node_handle_, this));
  }

  if (options.use_camera) {
    string topic = topics.image_topic;
    subscribers_[trajectory_id].push_back(
        SubscribeWithHandler<sensor_msgs::Image>(&Node::HandleImageMessage,
                                                 trajectory_id, topic,
                                                 &node_handle_, this));
  }
}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d_eskf()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_eskf_options();
  }
  return false;
}

bool Node::ValidateTopicNames(
    const ::cartographer_ros_msgs::SensorTopics& topics,
    const TrajectoryOptions& options) {
  for (const std::string& topic : ComputeExpectedTopics(options, topics)) {
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

bool Node::HandleStartTrajectory(
    ::cartographer_ros_msgs::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::StartTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  TrajectoryOptions options;
  if (!FromRosMessage(request.options, &options) ||
      !ValidateTrajectoryOptions(options)) {
    LOG(ERROR) << "Invalid trajectory options.";
    return false;
  }
  if (!ValidateTopicNames(request.topics, options)) {
    LOG(ERROR) << "Invalid topics.";
    return false;
  }
  response.trajectory_id = AddTrajectory(options, request.topics);
  return true;
}

void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(ValidateTrajectoryOptions(options));
  AddTrajectory(options, DefaultSensorTopics());
}

std::unordered_set<string> Node::ComputeDefaultTopics(
    const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  return ComputeExpectedTopics(options, DefaultSensorTopics());
}

int Node::AddOfflineTrajectory(
    const std::unordered_set<string>& expected_sensor_ids,
    const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  is_active_trajectory_[trajectory_id] = true;
  return trajectory_id;
}

bool Node::HandleFinishTrajectory(
    ::cartographer_ros_msgs::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  const int trajectory_id = request.trajectory_id;
  if (is_active_trajectory_.count(trajectory_id) == 0) {
    LOG(INFO) << "Trajectory_id " << trajectory_id << " is not created yet.";
    return false;
  }
  if (!is_active_trajectory_[trajectory_id]) {
    LOG(INFO) << "Trajectory_id " << trajectory_id
              << " has already been finished.";
    return false;
  }

  // Shutdown the subscribers of this trajectory.
  for (auto& entry : subscribers_[trajectory_id]) {
    entry.shutdown();
    subscribed_topics_.erase(entry.getTopic());
    LOG(INFO) << "Shutdown the subscriber of [" << entry.getTopic() << "]";
  }
  CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  is_active_trajectory_[trajectory_id] = false;
  return true;
}

bool Node::HandleWriteState(
    ::cartographer_ros_msgs::WriteState::Request& request,
    ::cartographer_ros_msgs::WriteState::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.SerializeState(request.filename);
  return true;
}

void Node::FinishAllTrajectories() {
  carto::common::MutexLocker lock(&mutex_);
  for (auto& entry : is_active_trajectory_) {
    const int trajectory_id = entry.first;
    if (entry.second) {
      map_builder_bridge_.FinishTrajectory(trajectory_id);
      entry.second = false;
    }
  }
}

void Node::FinishTrajectory(const int trajectory_id) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(is_active_trajectory_.at(trajectory_id));
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  is_active_trajectory_[trajectory_id] = false;
}

void Node::RunFinalOptimization() {
  {
    carto::common::MutexLocker lock(&mutex_);
    for (const auto& entry : is_active_trajectory_) {
      CHECK(!entry.second);
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_.RunFinalOptimization();
}

void Node::HandleOdometryMessage(const int trajectory_id,
                                 const string& sensor_id,
                                 const nav_msgs::Odometry::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleImageMessage(const int trajectory_id,
                              const string& sensor_id,
                              const sensor_msgs::Image::ConstPtr& msg) {
  std::cout << "Node::HandleImageMessage(), sensor_id" << sensor_id << std::endl;
  carto::common::MutexLocker lock(&mutex_);
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  sensor_bridge_ptr->HandleImageMessage(sensor_id, msg);
}

void Node::HandleImuMessage(const int trajectory_id, const string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

void Node::HandleMultiEchoLaserScanMessage(
    int trajectory_id, const string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Node::HandlePointCloud2Message(
    const int trajectory_id, const string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if(msg->header.stamp.toSec() > prev_stamp) {
    map_builder_bridge_.sensor_bridge(trajectory_id)
        ->HandlePointCloud2Message(sensor_id, msg);
        prev_stamp = msg->header.stamp.toSec();
  } else {

  }
}

void Node::SerializeState(const string& filename) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.SerializeState(filename);
}

void Node::LoadMap(const std::string& map_filename) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.LoadMap(map_filename);
}

}  // namespace cartographer_ros
