// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef NON_PLANAR_OBSTACLE_LAYER__OBSERVATION_SOURCE_HPP_
#define NON_PLANAR_OBSTACLE_LAYER__OBSERVATION_SOURCE_HPP_

#include <Eigen/Dense>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace non_planar_obstacle_layer
{

using std::placeholders::_1;
using namespace std::chrono_literals;

class ObservationSourceBase {
public:
  virtual ~ObservationSourceBase() {};

  virtual void clear_perception() = 0;
  virtual void insert_perception_in_pc(
    double min_x, double max_x, double min_y, double max_y, double min_z, double max_z,
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, double resolution) = 0;
};

template<class T>
class ObservationSource : public ObservationSourceBase
{
public:
  ObservationSource(
    const std::string & plugin_name, const std::string & name, rclcpp_lifecycle::LifecycleNode::SharedPtr node)
  : name_(name)
  {
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string topic;
    if (!node->has_parameter(plugin_name + "." + name + ".topic")) {
      node->declare_parameter(plugin_name + "." + name + ".topic", "/perception");
    }
    node->get_parameter(plugin_name + "." + name + ".topic", topic);

    RCLCPP_INFO(node->get_logger(), "\t\tTopic = [%s]", topic.c_str());

    percept_sub_ = node->create_subscription<T>(
      topic, rclcpp::SensorDataQoS(),
      std::bind(&ObservationSource::perception_callback, this, _1));
  }

  void set_last_perception(T & last_perception) {
    if (last_perception_ == nullptr) {
      last_perception_ = std::make_unique<T>();
    }
    *last_perception_ = last_perception;
  }

  virtual void clear_perception() {
    last_perception_ = nullptr;
  }

  bool get_transformation(
    const std::string & target_frame, std::string source_frame,
    const rclcpp::Time & time, Eigen::Affine3d & transform)
  {
    try {
      geometry_msgs::msg::TransformStamped sensor2map;
      sensor2map = tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2_ros::fromRclcpp(time),
        tf2::Duration(200ms));
      
      transform = tf2::transformToEigen(sensor2map);

      return true;
    } catch (tf2::TransformException& ex) {
      return false;
    }
  }

public:
  typename T::UniquePtr last_perception_;

protected:
  const std::string name_;
  typename rclcpp::Subscription<T>::SharedPtr percept_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void perception_callback(typename T::UniquePtr msg) {
    last_perception_ = std::move(msg);
  }
};


class LaserObservationSource : public ObservationSource<sensor_msgs::msg::LaserScan>
{
public:
  LaserObservationSource(
    const std::string & plugin_name, const std::string & name, rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  void insert_perception_in_pc(
    double min_x, double max_x, double min_y, double max_y, double min_z, double max_z,
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, double resolution);
};

class PointCloudObservationSource : public ObservationSource<sensor_msgs::msg::LaserScan>
{
public:
  PointCloudObservationSource(
    const std::string & plugin_name, const std::string & name, rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  void insert_perception_in_pc(
    double min_x, double max_x, double min_y, double max_y, double min_z, double max_z,
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, double resolution);
};

}  // namespace non_planar_obstacle_layer

#endif  // NON_PLANAR_OBSTACLE_LAYER__OBSERVATION_SOURCE_HPP_
