configuration:
  software:
    octomap_server:
      params:
        # =============================================================
        # Odom-frame servers
        # =============================================================

        front_lidar:
          octomap_server_node:
            publish_3d_map_period: 4.0
            publish_2d_period: 4.0
            segmented_topics:
              # Published while tilting
              - nonground_topic: "/laser_clouds/preprocessed_z_filtered"
                ground_topic: "/no_floor_grid/laser_clouds/floor"
                nonmarking_nonground_topic: "/laser_clouds/raw_z_filtered"
                sensor_origin_frame_id: "laser_mount"
              # Published while planar only
              - nonground_topic: "/scans/planar_only_cloud"
                sensor_origin_frame_id: "laser_mount"
            sensor_model:
              hit: 0.5498339973124778
              miss: 0.23147521650098246
              min: 0.5
              max: 0.9168273035060777
            # 60s max. 12s expiry at 3 hits, 39s at 6 hits.
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
            sensor_model:
              hit: 0.5041665702187285
              miss: 0.43782349911420193
              min: 0.5
              max: 0.8807970779778824
            # 30s max. 5s expiry at ~1s of hits, 10s at ~2s.
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
            publish_3d_map_period: 4.0
            publish_2d_period: 4.0
            segmented_topics:
              - nonground_topic: "/lidars/top_front/segmentation/obstacle_cloud"
                ground_topic: "/lidars/top_front/segmentation/floor_cloud"
                nonmarking_nonground_topic: "/lidars/top_front/driver/cloud"
                sensor_origin_frame_id: "top_front_laser_parallel_to_base"
            sensor_model:
              hit_per_second: 0.598687660112452
              miss_per_second: 0.06496916912866402
              min: 0.5
              max: 0.7685247834990176
            # 60s max. 12s expiry at 15 hits, 39s at 30 hits.
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
            skip_count: 1
            publish_3d_map_period: 4.0
            publish_2d_period: 4.0
            segmented_topics:
              - nonground_topic: "/scans/top/velodyne_points_with_invalid"
                sensor_origin_frame_id: "top_laser_nominal"
            sensor_model:
              hit: 0.5099986668799654
              miss: 0.401312339887548
              min: 0.5
              max: 0.9168273035060777
            # 60s max. 12s expiry at 15 hits, 39s at 30 hits.
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
            publish_3d_map_period: 4.0
            publish_2d_period: 4.0
            segmented_topics:
              - nonground_topic: "/lidars/corner_lidars/front_right/segmentation/obstacle_cloud"
                sensor_origin_frame_id: "front_right_lidar_parallel_to_base"
              - nonground_topic: "/lidars/corner_lidars/back_left/segmentation/obstacle_cloud"
                sensor_origin_frame_id: "back_left_lidar_parallel_to_base"
            sensor_model:
              hit_per_second: 0.598687660112452
              miss_per_second: 0.06496916912866402
              min: 0.5
              max: 0.7685247834990176
            # 60s max. 12s expiry at 15 hits, 39s at 30 hits.
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
            skip_count: 1
            publish_3d_map_period: 4.0
            publish_2d_period: 4.0
            segmented_topics:
              - nonground_topic: "/depthcam_low_front/depth/obstacles"
              - nonground_topic: "/depthcam_low_back/depth/obstacles"
            sensor_model:
              hit_per_second: 0.598687660112452
              miss_per_second: 0.06496916912866402
              min: 0.5
              max: 0.7685247834990176
            # 60s max. 12s expiry at 15 hits, 39s at 30 hits.
            expiry:
              at_negative_infinity: 3.0
              at_positive_infinity: 60.0
              x1: 15
              at_x1: 12
              x2: 30
              at_x2: 39
              free_space: 60.0

        # =============================================================
        # Map-frame servers
        # =============================================================

        map:
          front_lidar:
            octomap_server_node:
              segmented_topics:
                # Published while tilting
                - nonground_topic: "/laser_clouds/preprocessed_z_filtered"
                  ground_topic: "/no_floor_grid/laser_clouds/floor"
                  nonmarking_nonground_topic: "/laser_clouds/raw_z_filtered"
                  sensor_origin_frame_id: "laser_mount"
                # Published while planar only
                - nonground_topic: "/scans/planar_only_cloud"
                  sensor_origin_frame_id: "laser_mount"
              sensor_model:
                hit: 0.574442516812
                miss: 0.354343693774
                min: 0.231475216501
                max: 0.973403006423
              # 10min max. 12s expiry at 3 hits, 300s at 6 hits.
              expiry:
                at_negative_infinity: 3.0
                at_positive_infinity: 600.0
                x1: 3
                at_x1: 12.0
                x2: 6
                at_x2: 300.0
                free_space: 300.0

          top_front_lidar:
            octomap_server_node:
              skip_count: 4
              segmented_topics:
                - nonground_topic: "/lidars/top_front/segmentation/obstacle_cloud"
                  ground_topic: "/lidars/top_front/segmentation/floor_cloud"
                  nonmarking_nonground_topic: "/lidars/top_front/driver/cloud"
                  sensor_origin_frame_id: "top_front_laser_parallel_to_base"
              sensor_model:
                hit_per_second: 0.6456563062257954
                miss_per_second: 0.23147521650098246
                min: 0.231475216501
                max: 0.973403006423
              # 10min max. 12s expiry at 3 hits, 300s at 6 hits.
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
              skip_count: 4
              segmented_topics:
                - nonground_topic: "/scans/top/velodyne_points_with_invalid"
                  sensor_origin_frame_id: "top_laser_nominal"
              sensor_model:
                hit: 0.574442516812
                miss: 0.354343693774
                min: 0.231475216501
                max: 0.973403006423
              # 10min max. 12s expiry at 3 hits, 300s at 6 hits.
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
              skip_count: 4
              segmented_topics:
                - nonground_topic: "/lidars/corner_lidars/front_right/segmentation/obstacle_cloud"
                  sensor_origin_frame_id: "front_right_lidar_parallel_to_base"
                - nonground_topic: "/lidars/corner_lidars/back_left/segmentation/obstacle_cloud"
                  sensor_origin_frame_id: "back_left_lidar_parallel_to_base"
              sensor_model:
                hit_per_second: 0.6456563062257954
                miss_per_second: 0.23147521650098246
                min: 0.231475216501
                max: 0.973403006423
              # 10min max. 12s expiry at 3 hits, 300s at 6 hits.
              expiry:
                at_negative_infinity: 3.0
                at_positive_infinity: 600.0
                x1: 3
                at_x1: 12.0
                x2: 6
                at_x2: 300.0
                free_space: 300.0

          # Depth cameras: shorter expiry than lidars in map frame due to
          # limited FoV clearing.

          proximity:
            octomap_server_node:
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
              sensor_model:
                hit: 0.5744425168116589
                miss: 0.08317269649392234
                min: 0.23147521650098246
                max: 0.9734030064231342
              # 30s max. 5s expiry at ~1s of hits, 10s at ~2s.
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
              skip_count: 9
              segmented_topics:
                - nonground_topic: "/depthcam_low_front/depth/obstacles"
                - nonground_topic: "/depthcam_low_back/depth/obstacles"
              sensor_model:
                hit: 0.5744425168116589
                miss: 0.08317269649392234
                min: 0.23147521650098246
                max: 0.9734030064231342
              # 10min max. 12s expiry at 3 hits, 300s at 6 hits.
              # Low obstacles leave the FoV as the robot passes but no other
              # sensor can observe them, so long memory is needed.
              expiry:
                at_negative_infinity: 3.0
                at_positive_infinity: 600.0
                x1: 3
                at_x1: 12.0
                x2: 6
                at_x2: 300.0
                free_space: 300.0
