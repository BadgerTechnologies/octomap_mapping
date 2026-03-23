configuration:
  software:
    octomap_server:
      params:
        # odom-frame servers do not run in a frame-id namespace for historical reasons
        front_lidar:
          octomap_server_node:
            # Default parameters for octomap_server for front lidar
            publish_3d_map_period: 4.0
            publish_2d_period: 4.0
            segmented_topics:
              # These topics are only published while tilting
              - nonground_topic: "/laser_clouds/preprocessed_z_filtered"
                ground_topic: "/no_floor_grid/laser_clouds/floor"
                nonmarking_nonground_topic: "/laser_clouds/raw_z_filtered"
                sensor_origin_frame_id: "laser_mount"
              # This topic is only published while planar only
              - nonground_topic: "/scans/planar_only_cloud"
                sensor_origin_frame_id: "laser_mount"
            # Use values appropriate for a short-term memory (<60 seconds)
            # after 3 hits, the expiry is 12 seconds
            # after 6 hits, the expiry is 39 seconds
            sensor_model:
              hit: 0.5498339973124778
              miss: 0.23147521650098246
              min: 0.5
              max: 0.9168273035060777
            expiry:
              at_negative_infinity: 3.0
              at_positive_infinity: 60.0
              x1: 3
              at_x1: 12
              x2: 6
              at_x2: 39
              free_space: 60.0
        proximity:
          octomap_server_node:
            # Default parameters for octomap_server for proximity depth cameras
            skip_count: 0
            segmented_topics:
              - nonground_topic: "/depthcam_proximity_front/depth_nofloor_nocliff/points"
                ground_topic: "/depthcam_proximity_front/depth_floor/points"
                nonmarking_nonground_topic: "/depthcam_proximity_front/depth_deskewed/points"
              - nonground_topic: "/depthcam_proximity_back/depth_nofloor_nocliff/points"
                ground_topic: "/depthcam_proximity_back/depth_floor/points"
                nonmarking_nonground_topic: "/depthcam_proximity_back/depth_deskewed/points"
              - nonground_topic: "/depthcam_proximity_front2/depth_nofloor_nocliff/points"
                ground_topic: "/depthcam_proximity_front2/depth_floor/points"
                nonmarking_nonground_topic: "/depthcam_proximity_front2/depth_deskewed/points"
              - nonground_topic: "/depthcam_proximity_back2/depth_nofloor_nocliff/points"
                ground_topic: "/depthcam_proximity_back2/depth_floor/points"
                nonmarking_nonground_topic: "/depthcam_proximity_back2/depth_deskewed/points"
            # Use values appropriate for a very short-term memory (<30 seconds)
            # after 15 hits (about a second) the expiry is only 5 seconds.
            # after 30 hits (about two seconds) the expiry is 10 seconds
            sensor_model:
              hit: 0.5041665702187285
              miss: 0.43782349911420193
              min: 0.5
              max: 0.8807970779778824
            expiry:
              at_negative_infinity: 3.0
              at_positive_infinity: 30.0
              x1: 15
              at_x1: 5
              x2: 30
              at_x2: 10
              free_space: 30.0
        top_front_lidar:
          octomap_server_node:
            # Default parameters for octomap_server for top front lidar
            skip_count: 0
            publish_3d_map_period: 4.0
            publish_2d_period: 4.0
            # Essentially publish on every sensor update, set this much lower
            # than the maximum rate, which is 20Hz
            publish_3d_map_update_period: 0.02
            segmented_topics:
              - nonground_topic: "/lidars/top_front/segmentation/obstacle_cloud"
                ground_topic: "/lidars/top_front/segmentation/floor_cloud"
                nonmarking_nonground_topic: "/lidars/top_front/driver/cloud"
                sensor_origin_frame_id: "top_front_laser_parallel_to_base"
            # Use values appropriate for a short-term memory (<60 seconds)
            # after 15 hits, the expiry is 12 seconds
            # after 30 hits, the expiry is 39 seconds
            sensor_model:
              hit_per_second: 0.598687660112452
              miss_per_second: 0.06496916912866402
              min: 0.5
              max: 0.7685247834990176
            expiry:
              at_negative_infinity: 3.0
              at_positive_infinity: 60.0
              x1: 15
              at_x1: 12
              x2: 30
              at_x2: 39
              free_space: 60.0
        top_lidar:
          octomap_server_node:
            # Default parameters for octomap_server for top lidar
            skip_count: 1
            publish_3d_map_period: 4.0
            publish_2d_period: 4.0
            segmented_topics:
              - nonground_topic: "/scans/top/velodyne_points_with_invalid"
                sensor_origin_frame_id: "top_laser_nominal"
            # Use values appropriate for a short-term memory (<60 seconds)
            # after 15 hits, the expiry is 12 seconds
            # after 30 hits, the expiry is 39 seconds
            sensor_model:
              hit: 0.5099986668799654
              miss: 0.401312339887548
              min: 0.5
              max: 0.9168273035060777
            expiry:
              at_negative_infinity: 3.0
              at_positive_infinity: 60.0
              x1: 15
              at_x1: 12
              x2: 30
              at_x2: 39
              free_space: 60.0
        corner_lidars:
          octomap_server_node:
            # Default parameters for octomap_server for top front lidar
            skip_count: 0
            publish_3d_map_period: 4.0
            publish_2d_period: 4.0
            # Essentially publish on every sensor update, set this much lower
            # than the maximum rate, which is 20Hz
            publish_3d_map_update_period: 0.02
            segmented_topics:
              - nonground_topic: "/lidars/corner_lidars/front_right/segmentation/obstacle_cloud"
                sensor_origin_frame_id: "front_right_lidar_parallel_to_base"
              - nonground_topic: "/lidars/corner_lidars/back_left/segmentation/obstacle_cloud"
                sensor_origin_frame_id: "back_left_lidar_parallel_to_base"
            # Use values appropriate for a short-term memory (<60 seconds)
            # after 15 hits, the expiry is 12 seconds
            # after 30 hits, the expiry is 39 seconds
            sensor_model:
              hit_per_second: 0.598687660112452
              miss_per_second: 0.06496916912866402
              min: 0.5
              max: 0.7685247834990176
            expiry:
              at_negative_infinity: 3.0
              at_positive_infinity: 60.0
              x1: 15
              at_x1: 12
              x2: 30
              at_x2: 39
              free_space: 60.0
        low_cameras:
          octomap_server_node:
            # Default parameters for octomap_server for proximity depth cameras
            skip_count: 0
            segmented_topics:
              - nonground_topic: "/depthcam_low_front/depth/obstacles"
              - nonground_topic: "/depthcam_low_back/depth/obstacles"
        map:
          front_lidar:
            octomap_server_node:
              # Default parameters for octomap_server for front lidar in map frame
              segmented_topics:
                # These topics are only published while tilting
                - nonground_topic: "/laser_clouds/preprocessed_z_filtered"
                  ground_topic: "/no_floor_grid/laser_clouds/floor"
                  nonmarking_nonground_topic: "/laser_clouds/raw_z_filtered"
                  sensor_origin_frame_id: "laser_mount"
                # This topic is only published while planar only
                - nonground_topic: "/scans/planar_only_cloud"
                  sensor_origin_frame_id: "laser_mount"
              # Use values appropriate for a mid-term memory (~10 minutes).
              # Making the memory too long uses too much memory. The long-term mapping
              # needs to be done by a separate voxel statistics accumulation mode using
              # filesystem backed memory.
              # after 3 hits, the expiry is 12 seconds.
              # after 6 hits, the expiry is 300 seconds.
              # The sensor model is set to clear from the max to the min in 8 misses,
              # and to take 2 misses to get to the full expiration for free
              # space. This allows some level of sensor noise to be filtered here without
              # causing too much delay. The update rate of the global costmap does not need
              # to be super quick.
              # Have free space expire in 5 minutes, while the most burnt in obstacles
              # survive nearly 10 minutes. While this might cut off a path when part of
              # the static layer bleeds back through in the 3D costmap that uses this
              # octomap, we are less confident of free space, and it currently only decays
              # linearly over time. This can lead to too much free space being kept and not
              # enough obstacles allowing for goofy global plans. Allow the free space to
              # expire after 5 minutes so the static map re-appears.
              sensor_model:
                hit: 0.574442516812
                miss: 0.354343693774
                min: 0.231475216501
                max: 0.973403006423
              expiry:
                at_negative_infinity: 3.0
                at_positive_infinity: 600.0
                x1: 3
                at_x1: 12.0
                x2: 6
                at_x2: 300.0
                free_space: 300.0
          proximity:
            octomap_server_node:
              # Default parameters for octomap_server for proximity depth cameras in map frame
              skip_count: 9
              segmented_topics:
                - nonground_topic: "/depthcam_proximity_front/depth_nofloor_nocliff/points"
                  ground_topic: "/depthcam_proximity_front/depth_floor/points"
                - nonground_topic: "/depthcam_proximity_back/depth_nofloor_nocliff/points"
                  ground_topic: "/depthcam_proximity_back/depth_floor/points"
                - nonground_topic: "/depthcam_proximity_front2/depth_nofloor_nocliff/points"
                  ground_topic: "/depthcam_proximity_front2/depth_floor/points"
                - nonground_topic: "/depthcam_proximity_back2/depth_nofloor_nocliff/points"
                  ground_topic: "/depthcam_proximity_back2/depth_floor/points"
              # Use values appropriate for a very short-term memory (<30 seconds)
              # Historically the global costmap had zero memory of depth camera data.
              # Because # of the limited field of view, burning in too long causes problems
              # with clearing, as we will not approach the obstacle close enough to clear via
              # observation.
              # after 3 hits (about a second) the expiry is only 5 seconds.
              # after 6 hits (about two seconds) the expiry is 10 seconds
              sensor_model:
                hit: 0.5744425168116589
                miss: 0.08317269649392234
                min: 0.23147521650098246
                max: 0.9734030064231342
              expiry:
                at_negative_infinity: 3.0
                at_positive_infinity: 30.0
                x1: 3
                at_x1: 5
                x2: 6
                at_x2: 10
                free_space: 30.0
          low_cameras:
            octomap_server_node:
              # Default parameters for octomap_server for low depth cameras in map frame
              skip_count: 9
              segmented_topics:
                - nonground_topic: "/depthcam_low_front/depth/obstacles"
                - nonground_topic: "/depthcam_low_back/depth/obstacles"
              # Use values appropriate for a very short-term memory (<30 seconds)
              # after 3 hits (about a second) the expiry is only 5 seconds.
              # after 6 hits (about two seconds) the expiry is 10 seconds
              # Even though we have a better view of our surroundings with this sensor,
              # we will get a lot of foot traffic with the low nature of these cameras
              # and therefore want to expire the data relatively quickly.
              sensor_model:
                hit: 0.5744425168116589
                miss: 0.08317269649392234
                min: 0.23147521650098246
                max: 0.9734030064231342
              expiry:
                at_negative_infinity: 3.0
                at_positive_infinity: 30.0
                x1: 3
                at_x1: 5
                x2: 6
                at_x2: 10
                free_space: 30.0
          top_front_lidar:
            octomap_server_node:
              # Default parameters for octomap_server for top front lidar in map frame
              skip_count: 4
              segmented_topics:
                - nonground_topic: "/lidars/top_front/segmentation/obstacle_cloud"
                  ground_topic: "/lidars/top_front/segmentation/floor_cloud"
                  nonmarking_nonground_topic: "/lidars/top_front/driver/cloud"
                  sensor_origin_frame_id: "top_front_laser_parallel_to_base"
              # Use values appropriate for a mid-term memory (~10 minutes).
              # Making the memory too long uses too much memory. The long-term mapping
              # needs to be done by a separate voxel statistics accumulation mode using
              # filesystem backed memory.
              # after 3 hits, the expiry is 12 seconds.
              # after 6 hits, the expiry is 300 seconds.
              # The sensor model is set to clear from the max to the min in 8 misses,
              # and to take 2 misses to get to the full expiration for free
              # space. This allows some level of sensor noise to be filtered here without
              # causing too much delay. The update rate of the global costmap does not need
              # to be super quick.
              # Have free space expire in 5 minutes, while the most burnt in obstacles
              # survive nearly 10 minutes. While this might cut off a path when part of
              # the static layer bleeds back through in the 3D costmap that uses this
              # octomap, we are less confident of free space, and it currently only decays
              # linearly over time. This can lead to too much free space being kept and not
              # enough obstacles allowing for goofy global plans. Allow the free space to
              # expire after 5 minutes so the static map re-appears.
              sensor_model:
                hit_per_second: 0.6456563062257954
                miss_per_second: 0.23147521650098246
                min: 0.231475216501
                max: 0.973403006423
              expiry:
                at_negative_infinity: 3.0
                at_positive_infinity: 600.0
                x1: 3
                at_x1: 12.0
                x2: 6
                at_x2: 300.0
                free_space: 300.0
          top_lidar:
            octomap_server_node:
              # Default parameters for octomap_server for top lidar in map frame
              skip_count: 4
              segmented_topics:
                - nonground_topic: "/scans/top/velodyne_points_with_invalid"
                  sensor_origin_frame_id: "top_laser_nominal"
              # Use values appropriate for a mid-term memory (~10 minutes).
              # Making the memory too long uses too much memory. The long-term mapping
              # needs to be done by a separate voxel statistics accumulation mode using
              # filesystem backed memory.
              # after 3 hits, the expiry is 12 seconds.
              # after 6 hits, the expiry is 300 seconds.
              # The sensor model is set to clear from the max to the min in 8 misses,
              # and to take 2 misses to get to the full expiration for free
              # space. This allows some level of sensor noise to be filtered here without
              # causing too much delay. The update rate of the global costmap does not need
              # to be super quick.
              # Have free space expire in 5 minutes, while the most burnt in obstacles
              # survive nearly 10 minutes. While this might cut off a path when part of
              # the static layer bleeds back through in the 3D costmap that uses this
              # octomap, we are less confident of free space, and it currently only decays
              # linearly over time. This can lead to too much free space being kept and not
              # enough obstacles allowing for goofy global plans. Allow the free space to
              # expire after 5 minutes so the static map re-appears.
              sensor_model:
                hit: 0.574442516812
                miss: 0.354343693774
                min: 0.231475216501
                max: 0.973403006423
              expiry:
                at_negative_infinity: 3.0
                at_positive_infinity: 600.0
                x1: 3
                at_x1: 12.0
                x2: 6
                at_x2: 300.0
                free_space: 300.0
          corner_lidars:
            octomap_server_node:
              # Default parameters for octomap_server for top front lidar in map frame
              skip_count: 4
              segmented_topics:
                - nonground_topic: "/lidars/corner_lidars/front_right/segmentation/obstacle_cloud"
                  sensor_origin_frame_id: "front_right_lidar_parallel_to_base"
                - nonground_topic: "/lidars/corner_lidars/back_left/segmentation/obstacle_cloud"
                  sensor_origin_frame_id: "back_left_lidar_parallel_to_base"
              # Use values appropriate for a mid-term memory (~10 minutes).
              # Making the memory too long uses too much memory. The long-term mapping
              # needs to be done by a separate voxel statistics accumulation mode using
              # filesystem backed memory.
              # after 3 hits, the expiry is 12 seconds.
              # after 6 hits, the expiry is 300 seconds.
              # The sensor model is set to clear from the max to the min in 8 misses,
              # and to take 2 misses to get to the full expiration for free
              # space. This allows some level of sensor noise to be filtered here without
              # causing too much delay. The update rate of the global costmap does not need
              # to be super quick.
              # Have free space expire in 5 minutes, while the most burnt in obstacles
              # survive nearly 10 minutes. While this might cut off a path when part of
              # the static layer bleeds back through in the 3D costmap that uses this
              # octomap, we are less confident of free space, and it currently only decays
              # linearly over time. This can lead to too much free space being kept and not
              # enough obstacles allowing for goofy global plans. Allow the free space to
              # expire after 5 minutes so the static map re-appears.
              sensor_model:
                hit_per_second: 0.6456563062257954
                miss_per_second: 0.23147521650098246
                min: 0.231475216501
                max: 0.973403006423
              expiry:
                at_negative_infinity: 3.0
                at_positive_infinity: 600.0
                x1: 3
                at_x1: 12.0
                x2: 6
                at_x2: 300.0
                free_space: 300.0
