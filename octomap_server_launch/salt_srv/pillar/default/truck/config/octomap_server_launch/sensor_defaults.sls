configuration:
  software:
    octomap_server:
      octomap_server_sensor_default:
        {# Default parameters for octomap_server used with sensor data #}
        height_map: false
        timed_map: true
        track_free_space: false
        defer_update_to_publish: true
        {# no need to compress map since it is maintained pruned already #}
        compress_map: false
        {# do not latch topics #}
        latch: false
        color_free:
          r: 1.0
          g: 0.5
          b: 1.0
          a: 0.2
        publish_free_space: true
        publish_3d_map_period: 2.0
        publish_3d_map_update_period: 0.066666666
        publish_2d_period: 2.0
        expire_time_delta: 1.0

      octomap_server_sensor_default_odom:
        {# Default parameters for octomap_server running in odom frame #}
        resolution: .05
        base_2d_distance_limit: 4.0
        base_height_limit: 2.05
        base_depth_limit: 0.0
        base_distance_limit_time_delta: 2.0

      octomap_server_sensor_default_map:
        {# Default parameters for octomap_server running in map frame #}
        resolution: .05
        base_height_limit: 3.00
        base_depth_limit: 0.0
        {# do not publish the full map very often, as it may be huge! #}
        publish_3d_map_period: 60.0
        publish_2d_period: 60.0
        publish_3d_map_update_period: 1.0
        expire_time_delta: 1.0
        base_distance_limit_time_delta: 2.0
        update_2d_distance_limit: 5.0
        track_free_space: true
        tree_depth: 24
