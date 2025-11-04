## ISWARD II – Faits vérifiés (extraits du dépôt et fichiers fournis)

### Version et modèle
- Fichier `version.yaml`:
  - version: v1.2.1.250814-0(release)
  - model: G30

### Lancements principaux
- `launch/isward_common.launch` (inclut selon flags):
  - sensor: `src/sensor-pkg/launch/sensor.launch`
  - chassis: `src/chassis-pkg/launch/chassis.launch`
  - location: `src/location-pkg/launch/location.launch`
  - scheduler: `src/scheduler-pkg/launch/scheduler.launch`
  - parking: `src/parking-pkg/launch/parking.launch`
  - mapping: `src/mapping-pkg/launch/mapping.launch`
  - navigation: `src/navigation-pkg/launch/navigation.launch`
  - vision: `src/vision-pkg/charger_pose_pkg/launch/charger_pose_pkg.launch`
  - exps: `src/exps_pkg/launch/exps_pkg.launch`
  - rosbag optionnel: `launch/include/rosbag.launch`

### Navigation (mower profile)
- `src/navigation-pkg/launch/navigation.launch` → `mower/launch/navigation.launch`:
  - TF statique: base_link → isward_link (0.35, 0, 0.093)
  - Inclus: `movebase_normal.launch`, `pointcloud2laserscan.launch`, `map_server.launch`
- `mower/launch/movebase_normal.launch`:
  - Charge params: `move_base_params.yaml`, `costmap_common_params_burger.yaml` (global/local), `global_costmap_params.yaml`, `local_costmap_params.yaml`, `coverage_planner_params.yaml`, `stanley_control_params.yaml`
  - base_global_planner: coverage_planner/CoveragePlanner
  - base_local_planner: stanley_controller/StanleyController
- Costmaps et plugins:
  - `costmap_2d/costmap_plugins.xml`: StaticLayer, ObstacleLayer, InflationLayer, VoxelLayer, ContractSensorLayer
  - `global_costmap_params.yaml`: Static + Obstacle + ContractSensor + Inflation
  - `local_costmap_params.yaml`: Obstacle + Inflation, rolling_window, 6×6 m
  - `costmap_common_params_burger.yaml`: footprint, sources `/scan`/`/depth/scan`/PointCloud2, static map topic `/map`
- `move_base_params.yaml` (extraits): controller_frequency: 20.0; planner_frequency: 0; recoveries: rotate_recovery, move_back_recovery; oscillation_distance: 0.2; oscillation_timeout: 45.0; controller_patience: 3.0; planner_patience: 30.0
- Stanley params: `stanley_control_params.yaml` (lateral_gain_p, xy_goal_tolerance, robot_radius, etc.) et `StanleyController.cfg` (dynamic_reconfigure: desired_linear_vel, max_linear_x, max_angular_z, …)

#### Détails complets des paramètres clés
- `mower/params/move_base_params.yaml`:
  - base_global_planner: coverage_planner/CoveragePlanner
  - base_local_planner: stanley_controller/StanleyController
  - controller_frequency: 20.0
  - planner_frequency: 0
  - max_planning_retries: 1
  - shutdown_costmaps: true
  - recovery_behavior_enabled: true
  - clearing_rotation_allowed: false
  - conservative_reset_dist: 3.0
  - planner_patience: 30.0; controller_patience: 3.0
  - oscillation_distance: 0.2; oscillation_timeout: 45.0
  - rotate_recovery: { max_vel_theta: 0.5, min_in_place_vel_theta: 0.3 }
  - move_back_recovery: { distance_backwards: 0.2, predict_distance: 0.3, time_for_backwards: 4.0, frequency: 20.0, lethal_cost: 254, linear_vel_x: 0.1 }

- `mower/params/global_costmap_params.yaml`:
  - global_frame: map; robot_base_frame: base_link
  - update_frequency: 2.0; publish_frequency: 2.0; transform_tolerance: 0.5
  - static_map: true; rolling_window: false; resolution: 0.06; track_unknown_space: true
  - plugins: StaticLayer, ObstacleLayer, ContractSensorLayer, InflationLayer
  - obstacles.footprint_clearing_enabled: false
  - inflation_layer: { cost_scaling_factor: 1, inflation_radius: 0.25 }
  - collision_layer: { enabled: true, topics: ["/collision"], base_to_front_distance: 0.45 }

- `mower/params/local_costmap_params.yaml`:
  - global_frame: map; robot_base_frame: base_link
  - update_frequency: 6.0; publish_frequency: 4.0; transform_tolerance: 0.35
  - static_map: false; rolling_window: true; width: 6; height: 6; resolution: 0.06; track_unknown_space: false
  - plugins: ObstacleLayer, InflationLayer
  - obstacles.footprint_clearing_enabled: true
  - inflation_layer: { cost_scaling_factor: 1, inflation_radius: 0.53 }

- `mower/params/costmap_common_params_burger.yaml`:
  - footprint: [[0.413,-0.227],[0.413,0.227],[-0.151,0.227],[-0.151,-0.227]]
  - transform_tolerance: 0.2; map_type: costmap; always_send_full_costmap: false; resolution: 0.06
  - obstacles: { enabled: true, combination_method: 1, observation_sources: depth_scan, footprint_clearing_enabled: false }
  - scan: topic /scan, sensor_frame laser_link, data_type LaserScan, clearing true, marking true, max_obstacle_height 2.0, min_obstacle_height 0.0, obstacle_range 2.5, raytrace_range 3.0, inf_is_valid true
  - depth_scan: topic /depth/scan, sensor_frame isward_link, data_type LaserScan, clearing true, marking true, max_obstacle_height 2.0, min_obstacle_height -0.1, obstacle_range 1.4, raytrace_range 1.5, inf_is_valid true
  - pointcloud: topic /isward/depth/points, data_type PointCloud2, clearing true, marking true, max_obstacle_height 2.0, min_obstacle_height 0.0, obstacle_range 2.0, raytrace_range 2.5
  - inflation_layer: { cost_scaling_factor: 1, inflation_radius: 0.25 }
  - static: { enabled: true, track_unknown_space: true, map_topic: /map, unknown_cost_value: -1, lethal_cost_threshold: 89 }

- `stanley_controller/cfg/StanleyController.cfg` (dynamic_reconfigure):
  - desired_linear_vel: [0.1, 2.0] (default 0.4)
  - rotate_to_heading_angular_vel: [0.2, 1.8] (default 1.0)
  - rotate_to_heading_yaw_diff: [0.05, 1.57] (default 0.09)
  - border_plan_rotate_yaw_diff: [0.2, 1.57] (default 0.5)
  - astar_plan_rotate_yaw_diff: [0.5, 1.57] (default 1.2)
  - PID_RATE_P/I/D/IMAX: defaults 0.5/0.01/0.1/0.2
  - ROTATE_P: default 0.5
  - max_linear_x: [0.6, 2.0] (default 0.6)
  - max_angular_z: [0.8, 2.0] (default 1.0)
  - dec_vel_dis: default 0.4
  - lookforward_distance: [0.2,1.0] (default 0.5)
  - border_lookforward_distance: [0.1,1.0] (default 0.2)
  - linear_constant: [0.2,1.0] (default 0.3)
  - high/slow/stop_velocity_offset_x: 0.6/0.4/0.2
  - high/slow/circle/stop_area_points_num_thresh: 10/148/300/148
  - last_record_area_points_num_thresh: [1,80] (default 1)

### Mapping
- `mapping-pkg/launch/mapping.launch` (inclut `mapping/launch/mapping.launch`).
- Services générés (dans `devel`): `mapping/Mapping` (mode 1/2), `mapping/loadmap`, `mapping/querymap`.
- `map_server/launch/map_server.launch`: args `/home/isward/.isward/map/map.yaml`.

### Capteurs caméra
- `sensor-pkg/launch/sensor.launch` inclut `astra_camera/launch/dabai_u3.launch` et `camera_checker/launch/camera_checker.launch`.
- `dabai_u3.launch` accepte `serial_number` (string) et de multiples flags d’activation (color/depth/IR/pointcloud).

### Châssis et IO
- BLE: `chassis-pkg/launch/hal_comm.hal_ble.launch` → node `hal_ble` (`/dev/tty_BLE`, 115200).
- MQTT: `chassis-pkg/launch/hal_comm.hal_remote.launch` → host `mqtt.xiaojia-tech.com.cn`, port `8331`; charge `secret.yaml`.
- LTE: `chassis-pkg/launch/hal_usim.launch` → `/dev/tty_LTE`, 115200. RSSI publié sur `/usim/rssi`.
- GNSS/RTCM: `chassis-pkg/launch/hal_gnss.hal_rtcm.launch` → `/dev/tty_RADIO`, config_io_index=13.
- Secrets: `chassis-pkg/launch/secret.yaml` contient `secret: [..16 clés..]` et `model: [G10X,G6,G10,G10H,G20,G20H,G30,G30H]`.

### Scheduler
- `scheduler-pkg/launch/functions.yaml`: drapeaux de fonctions (ex: `VISION_LOCATING:default=true`).
- `scheduler-pkg/launch/velocity.csv`: profils (v_lin, v_ang, blade_height, blade_power) avec séries d’accélération/décélération/constante/recul/virage/arrêt.

### Vision (charger_pose_pkg)
- Launch: `vision-pkg/charger_pose_pkg/launch/charger_pose_pkg.launch`
  - `enable_station:=true`
  - `charger_object_model_path`: `data/weights/detect_charger_station.onnx`
  - `point_model_path`: `data/weights/detect_point.onnx`
  - `save_image_dir`: `/home/isward/.isward/raw_charger`
  - `camera_and_charger_info.yaml`: matrices intrinsèques RGB/IR, `charger_side_length: 0.06`, `label_point` ROI
- Engines TensorRT observés: `~/.isward/vision/charger_obj/*.engine`, `charger_opt/*.engine`.

### Données runtime (~/.isward)
- `~/.isward/map/`: `map.yaml`, `map.pgm`, `gen_path/map.yaml|pgm`, `draft_map.info`, `work_map.info`, `record.info`.
- `~/.isward/coverage_planner/`: `global/map.png`, `sub_*/{map.png,coverage,planner,shadow_map.png}`, `zones/map.png`, `zones/zone`, `map_list`, `plan_info`.
- `~/.isward/nav/isward.urdf.xacro`.
- `~/.isward/vision/.../*.engine`.
- `~/.isward/bag/` (rosbags).
- PNG diagnostics: `error_*`, `can_not_reach_*`, `robot_sliped.png`, `last_recovery_behaviors.png`.
- Logs GNSS: `gnss_record.log`.

### Topics (extraits list fournis)
- Châssis: `/chassis/batteries`, `/chassis/fault`, `/chassis/gnss`, `/chassis/gnss_raw`, `/chassis/imu`, `/chassis/odometer`, `/chassis/odometer_total`, `/chassis/rtcm_stream`, `/chassis/sensor`, `/chassis/set_velocity`, `/chassis/trigger`
- Nav/Mapping: `/cmd_vel`, `/collision`, `/depth/scan`, `/map`, `/map_metadata`, `/last_valid_plan`, `/move_base/recovery_status`, `/robot_walk_path`
- Localisation et états: `/location`, `/location_state`, `/nav_node_init`, `/switch_location`, `/path_fusion`, `/path_signal`
- Action NaviCtrl: `/navi/cmd/{goal,cancel,status,feedback,result}`, `/navi/plan_path`
- Parking/Vision: `/parking/yaw`, `/parking_vision_init`
- Remote/Host/Keyboard: `/remote/iot`, `/remote_ctrl/{dev_to_remote,remote_to_dev}`, `/host_ctrl/{dev_to_host,host_to_dev}`, `/keyboard_ctrl/{dev_to_keyboard,keyboard_to_dev}`
- Système: `/usim/rssi`, `/rosout`, `/rosout_agg`, `/tf`, `/tf_static`, `/Mapping`, `/maping_status`, `/test/rslt`, `/error_code`, `/exps/obstacle_signal`, `/scheduler/{safe_with,state}`

### Auteurs (depuis package.xml)
- `coverage_planner`: maintainer Terry <zhoutaoterry@gmail.com> (BSD)
- `stanley_controller`: maintainer/author wuyx <527160730@qq.com> (BSD)
- `mapping`: maintainer tony-ws1 <shantizhan@163.com>
- `location-pkg`: maintainer terry <zhoutaoterry@gmail.com> (BSD)
- `astra_camera`: maintainer Mo Cun <mocun@orbbec.com> (Apache-2.0)


