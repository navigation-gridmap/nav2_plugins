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



#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2_ros/buffer_interface.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "non_planar_obstacle_layer/ObservationSource.hpp"

namespace non_planar_obstacle_layer
{

LaserObservationSource::LaserObservationSource(
  const std::string & plugin_name, const std::string & name, rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: ObservationSource(plugin_name, name, node)
{
}

void
LaserObservationSource::insert_perception_in_pc(
    double min_x, double max_x, double min_y, double max_y, double min_z, double max_z,
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, double resolution)
{
  Eigen::Affine3d sensor2map;
  if (get_transformation("map", last_perception_->header.frame_id,
    rclcpp::Time(last_perception_->header.stamp),
    sensor2map))
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_pc(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < last_perception_->ranges.size(); i++) {
      if (std::isnan(last_perception_->ranges[i])) {continue;}

      double angle = last_perception_->angle_min +
        static_cast<double>(i) * last_perception_->angle_increment;

      pcl::PointXYZ point;
      point.x = last_perception_->ranges[i] * cos(angle);
      point.y = last_perception_->ranges[i] * sin(angle);
      point.z = 0.0;

      input_pc->push_back(point);
    }

    // Reduce density
    pcl::PointCloud<pcl::PointXYZ>::Ptr reduced_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(input_pc);
    voxel_grid.setLeafSize(resolution, resolution, 10.0f);
    voxel_grid.filter(*reduced_pc);

    // Crop pc in base on coordinates
    Eigen::Vector4f minPoint(min_x, min_y, min_z, 1.0); // Minimum point coordinates (x, y, z, 1)
    Eigen::Vector4f maxPoint(max_x, max_y, max_z, 1.0);   // Maximum point coordinates (x, y, z, 1)
    pcl::CropBox<pcl::PointXYZ> cropBoxFilter;
    cropBoxFilter.setInputCloud(reduced_pc);
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);
    cropBoxFilter.filter(*reduced_pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*reduced_pc, *temp_pc, sensor2map);

    pc_out->insert(pc_out->end(), temp_pc->begin(), temp_pc->end());
  }
}

PointCloudObservationSource::PointCloudObservationSource(
  const std::string & plugin_name, const std::string & name, rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: ObservationSource(plugin_name, name, node)
{
}

void
PointCloudObservationSource::insert_perception_in_pc(
    double min_x, double max_x, double min_y, double max_y, double min_z, double max_z,
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, double resolution)
{
  Eigen::Affine3d sensor2map;
  if (get_transformation("map", last_perception_->header.frame_id,
    rclcpp::Time(last_perception_->header.stamp),
    sensor2map))
  {
    // Reduce density
    pcl::PointCloud<pcl::PointXYZ>::Ptr reduced_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(reduced_pc);
    voxel_grid.setLeafSize(resolution, resolution, 10.0f);
    voxel_grid.filter(*reduced_pc);

    // Crop pc in base on coordinates
    Eigen::Vector4f minPoint(min_x, min_y, min_z, 1.0); // Minimum point coordinates (x, y, z, 1)
    Eigen::Vector4f maxPoint(max_x, max_y, max_z, 1.0);   // Maximum point coordinates (x, y, z, 1)
    pcl::CropBox<pcl::PointXYZ> cropBoxFilter;
    cropBoxFilter.setInputCloud(reduced_pc);
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);
    cropBoxFilter.filter(*reduced_pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*reduced_pc, *temp_pc, sensor2map);

    pc_out->insert(pc_out->end(), temp_pc->begin(), temp_pc->end());
  }
}

}  // namespace non_planar_obstacle_layer

