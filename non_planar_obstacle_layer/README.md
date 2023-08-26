# Non-Planar Obstacle Layer

This plugin uses gridmap and different perception sources to detect obstacles in a non-planar environment.

Usage:

```
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "non_planar_obstacle_layer/NonPlanarObstacleLayer"
        enabled: True
        gridmap_topic: /grid_map_map
        min_z: -20.0
        max_z: 20.0
        min_obstacle_height: 0.4
        max_obstacle_height: 2.0
        observation_sources: ["front_scan"]
        front_scan:
          topic: "/front_laser/scan"
          type: "LaserScan"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
```

## Parameters

* `enabled` (boolean, true): If the plugin is enabled
* `gridmap_topic` (string, "grid_map_map"): The topic of the gridmap.
* `min_z` (double, -inf): The miniumn Z of the map elements.
* `max_z` (double, -inf): The maxiumn Z of the map elements.
* `min_obstacle_height` (double, 0.2): The minimun height above the surface of an obstacle.
* `max_obstacle_height` (double, 2.0): The maximun height above the surface of an obstacle.
* `bounds_size_x` (double, 40): This is the X size of the area to detect obstacles.
* `bounds_size_y` (double, 40): This is the Y size of the area to detect obstacles.
* `observation_sources` (string, ""): The perception sources to detect obstacles.
  * `type` (string, ""): The perception source type. Currently in [`LaserScan`, `PointCloud2`]
  * `topic` (string, "/perception"): The perception source topic.