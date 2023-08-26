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

#include "non_planar_obstacle_layer/non_planar_obstacle_layer.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using std::placeholders::_1;

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace non_planar_obstacle_layer
{

NonPlanarObstacleLayer::NonPlanarObstacleLayer()
: min_z_(-std::numeric_limits<double>::max()),
  max_z_(std::numeric_limits<double>::max()),
  min_obstacle_height_(0.2),
  max_obstacle_height_(2.0),
  bounds_size_x_(40.0),
  bounds_size_y_(40.0)
{
  gridmap_ = std::make_shared<grid_map::GridMap>();

  current_ = true;
}

void
NonPlanarObstacleLayer::onInitialize()
{
  auto node = node_.lock();

  RCLCPP_INFO(node->get_logger(), "Initializing NonPlanarObstacleLayer");

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("min_z", rclcpp::ParameterValue(min_z_));
  declareParameter("max_z", rclcpp::ParameterValue(max_z_));
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(min_obstacle_height_));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(max_obstacle_height_));
  declareParameter("bounds_size_x", rclcpp::ParameterValue(bounds_size_x_));
  declareParameter("bounds_size_y", rclcpp::ParameterValue(bounds_size_y_));
  declareParameter("laser_enabled", rclcpp::ParameterValue(false));
  declareParameter("pointcloud_enabled", rclcpp::ParameterValue(false));
  
  std::vector<std::string> observation_sources;
  declareParameter("observation_sources", rclcpp::ParameterValue(observation_sources));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "min_z", min_z_);
  node->get_parameter(name_ + "." + "max_z", max_z_);
  node->get_parameter(name_ + "." + "min_obstacle_height", min_obstacle_height_);
  node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + "." + "bounds_size_x", bounds_size_x_);
  node->get_parameter(name_ + "." + "bounds_size_y", bounds_size_y_);

  RCLCPP_INFO(node->get_logger(), "\tZ = [%lf - %lf]", min_z_, min_z_);
  RCLCPP_INFO(node->get_logger(), "\tHeight = [%lf - %lf]", min_obstacle_height_, max_obstacle_height_);
  RCLCPP_INFO(node->get_logger(), "\tBounds = [%lf - %lf]", bounds_size_x_, bounds_size_y_);

  std::string gridmap_topic("grid_map_map");
  declareParameter("gridmap_topic", rclcpp::ParameterValue(gridmap_topic));
  node->get_parameter(name_ + "." + "gridmap_topic", gridmap_topic);

  RCLCPP_INFO(node->get_logger(), "\tGridmap Topic = [%s]", gridmap_topic.c_str());

  sub_gridmap_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    gridmap_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&NonPlanarObstacleLayer::gridmap_callback, this, _1));

  declareParameter("observation_sources", rclcpp::ParameterValue(observation_sources));
  node->get_parameter(name_ + "." + "observation_sources", observation_sources);

  for (const auto & observation_source : observation_sources) {
    std::string observation_source_type;
    declareParameter(observation_source + ".type", rclcpp::ParameterValue(observation_source_type));
    node->get_parameter(name_ + "." + observation_source + ".type", observation_source_type);
 
    RCLCPP_INFO(node->get_logger(), "\tObservation Source = [%s]", observation_source.c_str());
    RCLCPP_INFO(node->get_logger(), "\tObservation Type = [%s]", observation_source_type.c_str());

    ObservationSourceBase * new_observation_source;
    if (observation_source_type == "LaserScan") {
      RCLCPP_INFO(node->get_logger(), "\t\tLaser");
      new_observation_source = new LaserObservationSource(name_, observation_source, node);
      observation_sources_.push_back(new_observation_source);
    } else if (observation_source_type == "PointCloud2") {
      RCLCPP_INFO(node->get_logger(), "\t\tPointCloud2");
      new_observation_source = new PointCloudObservationSource(name_, observation_source, node);
      observation_sources_.push_back(new_observation_source);
    } else {
       RCLCPP_WARN(node->get_logger(), "\tObservation source Type [%s] not valid", observation_source_type.c_str());
    }

    RCLCPP_INFO(
      node->get_logger(), "Created observation source [%s] (type [%s])",
      observation_source.c_str(), observation_source_type.c_str());
  }
  current_ = true;
}

void
NonPlanarObstacleLayer::gridmap_callback(const grid_map_msgs::msg::GridMap::ConstSharedPtr & msg)
{
  grid_map::GridMapRosConverter::fromMessage(*msg, *gridmap_);
}


void
NonPlanarObstacleLayer::onFootprintChanged()
{

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "NonPlanarObstacleLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

void
NonPlanarObstacleLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  *min_x = robot_x - bounds_size_x_ / 2.0;
  *max_x = robot_x + bounds_size_x_ / 2.0;
  *min_y = robot_y - bounds_size_y_ / 2.0;
  *max_y = robot_y + bounds_size_y_ / 2.0;
}

void
NonPlanarObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j,
  int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  double resolution = master_grid.getResolution();

  double min_x, max_x, min_y, max_y;
  master_grid.mapToWorld(min_i, min_j, min_x, min_y);
  master_grid.mapToWorld(max_i, max_j, max_x, max_y);

  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (const auto observation_source : observation_sources_) {
    observation_source->insert_perception_in_pc(
      min_x, max_x, min_y, max_y, min_z_, max_z_, obstacle_cloud, resolution);
  } 

  auto & gridpmap_pos = gridmap_->getPosition();

  for (const auto & point : obstacle_cloud->points) {
    const float & px = point.x;
    const float & py = point.y;
    const float & pz = point.z;
  
    grid_map::Position particle_pos(px, py);
    float elevation = gridmap_->atPosition("elevation", particle_pos  + gridpmap_pos);

    if (pz > (elevation + min_obstacle_height_) && pz < (elevation + max_obstacle_height_)) {
      unsigned int mx, my;
      if (!master_grid.worldToMap(px, py, mx, my)) {
        RCLCPP_INFO(logger_, "Computing map coords failed");
        continue;
      }

      unsigned int index = master_grid.getIndex(mx, my);
      master_array[index] = LETHAL_OBSTACLE;
    }
  }
}

}  // namespace non_planar_obstacle_layer

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(non_planar_obstacle_layer::NonPlanarObstacleLayer, nav2_costmap_2d::Layer)
