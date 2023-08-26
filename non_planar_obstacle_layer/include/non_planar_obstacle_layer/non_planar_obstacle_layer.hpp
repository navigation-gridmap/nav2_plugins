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

#ifndef NON_PLANAR_OBSTACLE_LAYER__NON_PLANAR_OBSTACLE_LAYER_HPP_
#define NON_PLANAR_OBSTACLE_LAYER__NON_PLANAR_OBSTACLE_LAYER_HPP_


#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

#include "non_planar_obstacle_layer/ObservationSource.hpp"

#include "rclcpp/rclcpp.hpp"

namespace non_planar_obstacle_layer
{

class NonPlanarObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  NonPlanarObstacleLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

   virtual void onFootprintChanged();
  virtual bool isClearable() {return false;}

private:
  double min_local_x_, max_local_x_, min_local_y_, max_local_y_, min_z_, max_z_;
  double min_obstacle_height_, max_obstacle_height_;
  double bounds_size_x_, bounds_size_y_;

  void gridmap_callback(const grid_map_msgs::msg::GridMap::ConstSharedPtr & msg);

  std::vector<ObservationSourceBase*> observation_sources_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_gridmap_;
  std::shared_ptr<grid_map::GridMap> gridmap_;
};

}  // namespace non_planar_obstacle_layer

#endif  // NON_PLANAR_OBSTACLE_LAYER__NON_PLANAR_OBSTACLE_LAYER_HPP_
