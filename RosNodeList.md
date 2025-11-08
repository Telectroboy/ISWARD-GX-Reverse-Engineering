isward@ubuntu:~$ for n in $(rosnode list); do
>   echo "=== $n ==="
>   rosnode info $n
>   echo ""
> done

=== /base_link_to_isward_link ===
--------------------------------------------------------------------------------
Node [/base_link_to_isward_link]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tf_static [tf2_msgs/TFMessage]

Subscriptions: None

Services: 
 * /base_link_to_isward_link/get_loggers
 * /base_link_to_isward_link/set_logger_level


contacting node http://localhost:36121/ ...
Pid: 5352
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (43569 - 127.0.0.1:35390) [11]
    * transport: TCPROS
 * topic: /tf_static
    * to: /move_base
    * direction: outbound (43569 - 127.0.0.1:35398) [9]
    * transport: TCPROS
 * topic: /tf_static
    * to: /scheduler
    * direction: outbound (43569 - 127.0.0.1:35412) [13]
    * transport: TCPROS
 * topic: /tf_static
    * to: /pointcloud_to_laserscan
    * direction: outbound (43569 - 127.0.0.1:35424) [12]
    * transport: TCPROS


=== /charger_location_cpp ===
--------------------------------------------------------------------------------
Node [/charger_location_cpp]
Publications: 
 * /error_code [std_msgs/Int32]
 * /parking_vision_init [std_msgs/Bool]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]
 * /touch_stop [std_msgs/Int32]

Subscriptions: None

Services: 
 * /charger_location_cpp/get_loggers
 * /charger_location_cpp/set_logger_level
 * /enable_charger_location


contacting node http://localhost:44709/ ...
Pid: 7089
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (57353 - 127.0.0.1:56348) [9]
    * transport: TCPROS
 * topic: /tf
    * to: /pointcloud_to_laserscan
    * direction: outbound (57353 - 127.0.0.1:56356) [10]
    * transport: TCPROS
 * topic: /tf
    * to: /move_base
    * direction: outbound (57353 - 127.0.0.1:56374) [12]
    * transport: TCPROS
 * topic: /tf
    * to: /scheduler
    * direction: outbound (57353 - 127.0.0.1:56388) [14]
    * transport: TCPROS
 * topic: /touch_stop
    * to: /parking
    * direction: outbound (57353 - 127.0.0.1:56362) [11]
    * transport: TCPROS
 * topic: /parking_vision_init
    * to: /scheduler
    * direction: outbound (57353 - 127.0.0.1:56384) [13]
    * transport: TCPROS


=== /check_camera_node ===
--------------------------------------------------------------------------------
Node [/check_camera_node]
Publications: 
 * /rosout [rosgraph_msgs/Log]

Subscriptions: None

Services: 
 * /check_camera_node/get_loggers
 * /check_camera_node/set_logger_level
 * /check_color
 * /check_depth


contacting node http://localhost:45895/ ...
Pid: 5194
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (42581 - 127.0.0.1:37238) [7]
    * transport: TCPROS


=== /hal_ble ===
--------------------------------------------------------------------------------
Node [/hal_ble]
Publications: 
 * /chassis/fault [interface_pkg/ChassisFault]
 * /error_code [std_msgs/Int32]
 * /host_ctrl/host_to_dev [interface_pkg/ChassisMessage]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /host_ctrl/dev_to_host [interface_pkg/ChassisMessage]

Services: 
 * /hal_ble/get_loggers
 * /hal_ble/set_logger_level


contacting node http://localhost:43287/ ...
Pid: 5196
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (39447 - 127.0.0.1:37194) [9]
    * transport: TCPROS
 * topic: /chassis/fault
    * to: /scheduler
    * direction: outbound (39447 - 127.0.0.1:37200) [13]
    * transport: TCPROS
 * topic: /host_ctrl/host_to_dev
    * to: /scheduler
    * direction: outbound (39447 - 127.0.0.1:37204) [12]
    * transport: TCPROS
 * topic: /host_ctrl/dev_to_host
    * to: /scheduler (http://localhost:39595/)
    * direction: inbound (34896 - localhost:38383) [15]
    * transport: TCPROS


=== /hal_chassis ===
--------------------------------------------------------------------------------
Node [/hal_chassis]
Publications: 
 * /chassis/batteries [interface_pkg/ChassisBatteries]
 * /chassis/fault [interface_pkg/ChassisFault]
 * /chassis/imu [sensor_msgs/Imu]
 * /chassis/odometer [interface_pkg/ChassisOdometer]
 * /chassis/odometer_total [interface_pkg/ChassisOdometer]
 * /chassis/sensor [interface_pkg/ChassisSensor]
 * /error_code [std_msgs/Int32]
 * /keyboard_ctrl/keyboard_to_dev [interface_pkg/ChassisStream]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /chassis/set_velocity [geometry_msgs/Twist]
 * /keyboard_ctrl/dev_to_keyboard [interface_pkg/ChassisStream]

Services: 
 * /chassis/ctrl_blade
 * /chassis/ctrl_device
 * /chassis/get_board_uuid
 * /chassis/get_board_version
 * /chassis/get_uuid
 * /chassis/get_version
 * /hal_chassis/get_loggers
 * /hal_chassis/set_logger_level


contacting node http://localhost:39453/ ...
Pid: 5195
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (59801 - 127.0.0.1:53870) [11]
    * transport: TCPROS
 * topic: /chassis/odometer
    * to: /location
    * direction: outbound (59801 - 127.0.0.1:53932) [17]
    * transport: TCPROS
 * topic: /chassis/odometer
    * to: /scheduler
    * direction: outbound (59801 - 127.0.0.1:53984) [24]
    * transport: TCPROS
 * topic: /chassis/odometer_total
    * to: /scheduler
    * direction: outbound (59801 - 127.0.0.1:53982) [12]
    * transport: TCPROS
 * topic: /chassis/imu
    * to: /parking
    * direction: outbound (59801 - 127.0.0.1:53894) [14]
    * transport: TCPROS
 * topic: /chassis/imu
    * to: /mapping_node
    * direction: outbound (59801 - 127.0.0.1:53918) [16]
    * transport: TCPROS
 * topic: /chassis/imu
    * to: /location
    * direction: outbound (59801 - 127.0.0.1:53942) [18]
    * transport: TCPROS
 * topic: /chassis/imu
    * to: /scheduler
    * direction: outbound (59801 - 127.0.0.1:53986) [25]
    * transport: TCPROS
 * topic: /chassis/batteries
    * to: /parking
    * direction: outbound (59801 - 127.0.0.1:53902) [15]
    * transport: TCPROS
 * topic: /chassis/batteries
    * to: /location
    * direction: outbound (59801 - 127.0.0.1:53958) [19]
    * transport: TCPROS
 * topic: /chassis/batteries
    * to: /scheduler
    * direction: outbound (59801 - 127.0.0.1:54000) [22]
    * transport: TCPROS
 * topic: /chassis/sensor
    * to: /scheduler
    * direction: outbound (59801 - 127.0.0.1:53968) [23]
    * transport: TCPROS
 * topic: /chassis/fault
    * to: /scheduler
    * direction: outbound (59801 - 127.0.0.1:54010) [27]
    * transport: TCPROS
 * topic: /keyboard_ctrl/keyboard_to_dev
    * to: /scheduler
    * direction: outbound (59801 - 127.0.0.1:54018) [30]
    * transport: TCPROS
 * topic: /keyboard_ctrl/dev_to_keyboard
    * to: /scheduler (http://localhost:39595/)
    * direction: inbound (34916 - localhost:38383) [26]
    * transport: TCPROS
 * topic: /chassis/set_velocity
    * to: /scheduler (http://localhost:39595/)
    * direction: inbound (34898 - localhost:38383) [21]
    * transport: TCPROS


=== /hal_gnss ===
--------------------------------------------------------------------------------
Node [/hal_gnss]
Publications: 
 * /chassis/fault [interface_pkg/ChassisFault]
 * /chassis/gnss [sensor_msgs/NavSatFix]
 * /chassis/gnss_raw [std_msgs/String]
 * /error_code [std_msgs/Int32]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /chassis/rtcm_stream [std_msgs/UInt8MultiArray]

Services: 
 * /chassis/get_rtk_version
 * /hal_gnss/get_loggers
 * /hal_gnss/set_logger_level


contacting node http://localhost:40065/ ...
Pid: 5198
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (58917 - 127.0.0.1:39504) [11]
    * transport: TCPROS
 * topic: /chassis/fault
    * to: /scheduler
    * direction: outbound (58917 - 127.0.0.1:39524) [16]
    * transport: TCPROS
 * topic: /chassis/gnss
    * to: /location
    * direction: outbound (58917 - 127.0.0.1:39512) [14]
    * transport: TCPROS
 * topic: /chassis/gnss
    * to: /scheduler
    * direction: outbound (58917 - 127.0.0.1:39518) [15]
    * transport: TCPROS
 * topic: /chassis/gnss_raw
    * to: /hal_rtcm
    * direction: outbound (58917 - 127.0.0.1:39508) [13]
    * transport: TCPROS
 * topic: /chassis/rtcm_stream
    * to: /hal_rtcm (http://localhost:34571/)
    * direction: inbound (46854 - localhost:60135) [12]
    * transport: TCPROS


=== /hal_remote ===
--------------------------------------------------------------------------------
Node [/hal_remote]
Publications: 
 * /remote/iot [std_msgs/Bool]
 * /remote_ctrl/remote_to_dev [interface_pkg/ChassisMessage]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /remote_ctrl/dev_to_remote [interface_pkg/ChassisMessage]

Services: 
 * /hal_remote/get_loggers
 * /hal_remote/set_logger_level


contacting node http://localhost:43857/ ...
Pid: 5197
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (39875 - 127.0.0.1:39692) [9]
    * transport: TCPROS
 * topic: /remote_ctrl/remote_to_dev
    * to: /scheduler
    * direction: outbound (39875 - 127.0.0.1:39708) [14]
    * transport: TCPROS
 * topic: /remote/iot
    * to: /scheduler
    * direction: outbound (39875 - 127.0.0.1:39694) [11]
    * transport: TCPROS
 * topic: /remote_ctrl/dev_to_remote
    * to: /scheduler (http://localhost:39595/)
    * direction: inbound (34910 - localhost:38383) [10]
    * transport: TCPROS


=== /hal_rtcm ===
--------------------------------------------------------------------------------
Node [/hal_rtcm]
Publications: 
 * /chassis/fault [interface_pkg/ChassisFault]
 * /chassis/rtcm_stream [std_msgs/UInt8MultiArray]
 * /error_code [std_msgs/Int32]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /chassis/gnss_raw [std_msgs/String]

Services: 
 * /chassis/gnss_rtcm_cmd
 * /chassis/gnss_rtcm_set
 * /hal_rtcm/get_loggers
 * /hal_rtcm/set_logger_level


contacting node http://localhost:34571/ ...
Pid: 5215
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (60135 - 127.0.0.1:46838) [11]
    * transport: TCPROS
 * topic: /chassis/fault
    * to: /scheduler
    * direction: outbound (60135 - 127.0.0.1:46870) [14]
    * transport: TCPROS
 * topic: /chassis/rtcm_stream
    * to: /hal_gnss
    * direction: outbound (60135 - 127.0.0.1:46854) [9]
    * transport: TCPROS
 * topic: /chassis/rtcm_stream
    * to: /scheduler
    * direction: outbound (60135 - 127.0.0.1:46860) [12]
    * transport: TCPROS
 * topic: /chassis/gnss_raw
    * to: /hal_gnss (http://localhost:40065/)
    * direction: inbound (39508 - localhost:58917) [13]
    * transport: TCPROS


=== /hal_usim ===
--------------------------------------------------------------------------------
Node [/hal_usim]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /usim/rssi [std_msgs/Byte]

Subscriptions: None

Services: 
 * /hal_usim/get_loggers
 * /hal_usim/set_logger_level
 * /usim/get_iccid
 * /usim/get_imei
 * /usim/get_imsi


contacting node http://localhost:35735/ ...
Pid: 5237
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (47283 - 127.0.0.1:50916) [11]
    * transport: TCPROS
 * topic: /usim/rssi
    * to: /scheduler
    * direction: outbound (47283 - 127.0.0.1:50932) [12]
    * transport: TCPROS


=== /isward/camera ===
--------------------------------------------------------------------------------
Node [/isward/camera]
Publications: 
 * /error_code [std_msgs/Int32]
 * /isward/color/camera_info [sensor_msgs/CameraInfo]
 * /isward/color/image_raw [sensor_msgs/Image]
 * /isward/depth/camera_info [sensor_msgs/CameraInfo]
 * /isward/depth/image_raw [sensor_msgs/Image]
 * /isward/depth/points [sensor_msgs/PointCloud2]
 * /isward/ir/camera_info [sensor_msgs/CameraInfo]
 * /isward/ir/image_raw [sensor_msgs/Image]
 * /plane [shape_msgs/Plane]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

Subscriptions: None

Services: 
 * /isward/camera/get_loggers
 * /isward/camera/set_camera_info
 * /isward/camera/set_logger_level
 * /isward/get_auto_white_balance
 * /isward/get_camera_info
 * /isward/get_camera_params
 * /isward/get_depth_exposure
 * /isward/get_depth_gain
 * /isward/get_depth_supported_video_modes
 * /isward/get_device_info
 * /isward/get_device_type
 * /isward/get_ir_exposure
 * /isward/get_ir_gain
 * /isward/get_ir_supported_video_modes
 * /isward/get_ldp_status
 * /isward/get_serial
 * /isward/get_uvc_exposure
 * /isward/get_uvc_gain
 * /isward/get_uvc_mirror
 * /isward/get_uvc_white_balance
 * /isward/get_version
 * /isward/reset_ir_exposure
 * /isward/reset_ir_gain
 * /isward/save_images
 * /isward/save_point_cloud_xyz
 * /isward/save_uvc_image
 * /isward/set_auto_white_balance
 * /isward/set_camera_info
 * /isward/set_depth_auto_exposure
 * /isward/set_depth_exposure
 * /isward/set_depth_gain
 * /isward/set_depth_mirror
 * /isward/set_fan
 * /isward/set_ir_auto_exposure
 * /isward/set_ir_exposure
 * /isward/set_ir_flood
 * /isward/set_ir_gain
 * /isward/set_ir_mirror
 * /isward/set_laser
 * /isward/set_ldp
 * /isward/set_uvc_auto_exposure
 * /isward/set_uvc_auto_white_balance
 * /isward/set_uvc_exposure
 * /isward/set_uvc_gain
 * /isward/set_uvc_mirror
 * /isward/set_uvc_white_balance
 * /isward/switch_ir_camera
 * /isward/toggle_depth
 * /isward/toggle_ir
 * /isward/toggle_uvc_camera
 * /query_devices


contacting node http://localhost:41425/ ...
Pid: 5193
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (43167 - 127.0.0.1:45540) [11]
    * transport: TCPROS
 * topic: /tf_static
    * to: /move_base
    * direction: outbound (43167 - 127.0.0.1:45548) [29]
    * transport: TCPROS
 * topic: /tf_static
    * to: /scheduler
    * direction: outbound (43167 - 127.0.0.1:45550) [23]
    * transport: TCPROS
 * topic: /tf_static
    * to: /pointcloud_to_laserscan
    * direction: outbound (43167 - 127.0.0.1:45554) [25]
    * transport: TCPROS
 * topic: /tf
    * to: /move_base
    * direction: outbound (43167 - 127.0.0.1:45560) [30]
    * transport: TCPROS
 * topic: /tf
    * to: /pointcloud_to_laserscan
    * direction: outbound (43167 - 127.0.0.1:45566) [31]
    * transport: TCPROS
 * topic: /tf
    * to: /scheduler
    * direction: outbound (43167 - 127.0.0.1:45582) [24]
    * transport: TCPROS


=== /location ===
--------------------------------------------------------------------------------
Node [/location]
Publications: 
 * /location [geometry_msgs/PoseStamped]
 * /location_state [std_msgs/UInt8MultiArray]
 * /path_fusion [nav_msgs/Path]
 * /path_signal [nav_msgs/Path]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]

Subscriptions: 
 * /chassis/batteries [interface_pkg/ChassisBatteries]
 * /chassis/gnss [sensor_msgs/NavSatFix]
 * /chassis/imu [sensor_msgs/Imu]
 * /chassis/odometer [interface_pkg/ChassisOdometer]
 * /switch_location [std_msgs/Bool]

Services: 
 * /location/get_loggers
 * /location/set_logger_level


contacting node http://localhost:45309/ ...
Pid: 5266
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (33041 - 127.0.0.1:43388) [11]
    * transport: TCPROS
 * topic: /location_state
    * to: /move_base
    * direction: outbound (33041 - 127.0.0.1:43420) [16]
    * transport: TCPROS
 * topic: /location_state
    * to: /scheduler
    * direction: outbound (33041 - 127.0.0.1:43422) [19]
    * transport: TCPROS
 * topic: /tf
    * to: /move_base
    * direction: outbound (33041 - 127.0.0.1:43396) [17]
    * transport: TCPROS
 * topic: /tf
    * to: /pointcloud_to_laserscan
    * direction: outbound (33041 - 127.0.0.1:43412) [9]
    * transport: TCPROS
 * topic: /tf
    * to: /scheduler
    * direction: outbound (33041 - 127.0.0.1:43416) [20]
    * transport: TCPROS
 * topic: /switch_location
    * to: /scheduler (http://localhost:39595/)
    * direction: inbound (34880 - localhost:38383) [21]
    * transport: TCPROS
 * topic: /chassis/gnss
    * to: /hal_gnss (http://localhost:40065/)
    * direction: inbound (39512 - localhost:58917) [12]
    * transport: TCPROS
 * topic: /chassis/odometer
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (53932 - localhost:59801) [13]
    * transport: TCPROS
 * topic: /chassis/imu
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (53942 - localhost:59801) [14]
    * transport: TCPROS
 * topic: /chassis/batteries
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (53958 - localhost:59801) [15]
    * transport: TCPROS


=== /map_server ===
--------------------------------------------------------------------------------
Node [/map_server]
Publications: 
 * /map [nav_msgs/OccupancyGrid]
 * /map_metadata [nav_msgs/MapMetaData]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: None

Services: 
 * /change_map
 * /map_server/get_loggers
 * /map_server/set_logger_level
 * /static_map


contacting node http://localhost:39719/ ...
Pid: 5397
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (57633 - 127.0.0.1:46178) [10]
    * transport: TCPROS


=== /mapping_node ===
--------------------------------------------------------------------------------
Node [/mapping_node]
Publications: 
 * /error_code [std_msgs/Int32]
 * /maping_status [interface_pkg/MappingResult]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /Mapping [unknown type]
 * /chassis/imu [sensor_msgs/Imu]

Services: 
 * /launch_map_module
 * /mapping_node/get_loggers
 * /mapping_node/set_logger_level


contacting node http://localhost:45409/ ...
Pid: 5332
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (53825 - 127.0.0.1:41616) [11]
    * transport: TCPROS
 * topic: /chassis/imu
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (53918 - localhost:59801) [13]
    * transport: TCPROS


=== /move_base ===
--------------------------------------------------------------------------------
Node [/move_base]
Publications: 
 * /cmd_vel [geometry_msgs/Twist]
 * /last_valid_plan [nav_msgs/Path]
 * /move_base/recovery_status [move_base_msgs/RecoveryStatus]
 * /nav_node_init [std_msgs/Bool]
 * /robot_walk_path [nav_msgs/Path]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /collision [std_msgs/Bool]
 * /location_state [std_msgs/UInt8MultiArray]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

Services: 
 * /move_base/get_loggers
 * /move_base/set_logger_level


contacting node http://localhost:42511/ ...
Pid: 5375
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (59579 - 127.0.0.1:56940) [11]
    * transport: TCPROS
 * topic: /cmd_vel
    * to: /scheduler
    * direction: outbound (59579 - 127.0.0.1:56962) [17]
    * transport: TCPROS
 * topic: /nav_node_init
    * to: /scheduler
    * direction: outbound (59579 - 127.0.0.1:56952) [12]
    * transport: TCPROS
 * topic: /tf
    * to: /location (http://localhost:45309/)
    * direction: inbound (43396 - localhost:33041) [14]
    * transport: TCPROS
 * topic: /tf
    * to: /isward/camera (http://localhost:41425/)
    * direction: inbound (45560 - localhost:43167) [16]
    * transport: TCPROS
 * topic: /tf
    * to: /scheduler (http://localhost:39595/)
    * direction: inbound (34932 - localhost:38383) [21]
    * transport: TCPROS
 * topic: /tf
    * to: /charger_location_cpp (http://localhost:44709/)
    * direction: inbound (56374 - localhost:57353) [22]
    * transport: TCPROS
 * topic: /tf_static
    * to: /base_link_to_isward_link (http://localhost:36121/)
    * direction: inbound (35398 - localhost:43569) [13]
    * transport: TCPROS
 * topic: /tf_static
    * to: /isward/camera (http://localhost:41425/)
    * direction: inbound (45548 - localhost:43167) [18]
    * transport: TCPROS
 * topic: /collision
    * to: /scheduler (http://localhost:39595/)
    * direction: inbound (34894 - localhost:38383) [19]
    * transport: TCPROS
 * topic: /location_state
    * to: /location (http://localhost:45309/)
    * direction: inbound (43420 - localhost:33041) [15]
    * transport: TCPROS


=== /parking ===
--------------------------------------------------------------------------------
Node [/parking]
Publications: 
 * /cmd_vel [geometry_msgs/Twist]
 * /parking/yaw [std_msgs/Float64]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /chassis/batteries [interface_pkg/ChassisBatteries]
 * /chassis/imu [sensor_msgs/Imu]
 * /touch_stop [std_msgs/Int32]

Services: 
 * /parking/ctrl
 * /parking/get_loggers
 * /parking/set_logger_level


contacting node http://localhost:45289/ ...
Pid: 5325
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (46063 - 127.0.0.1:47240) [11]
    * transport: TCPROS
 * topic: /cmd_vel
    * to: /scheduler
    * direction: outbound (46063 - 127.0.0.1:47256) [9]
    * transport: TCPROS
 * topic: /touch_stop
    * to: /charger_location_cpp (http://localhost:44709/)
    * direction: inbound (56362 - localhost:57353) [15]
    * transport: TCPROS
 * topic: /chassis/imu
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (53894 - localhost:59801) [12]
    * transport: TCPROS
 * topic: /chassis/batteries
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (53902 - localhost:59801) [10]
    * transport: TCPROS


=== /pointcloud_to_laserscan ===
--------------------------------------------------------------------------------
Node [/pointcloud_to_laserscan]
Publications: 
 * /depth/scan [sensor_msgs/LaserScan]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]

Services: 
 * /pointcloud_to_laserscan/get_loggers
 * /pointcloud_to_laserscan/list
 * /pointcloud_to_laserscan/load_nodelet
 * /pointcloud_to_laserscan/set_logger_level
 * /pointcloud_to_laserscan/unload_nodelet


contacting node http://localhost:46083/ ...
Pid: 5382
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (56397 - 127.0.0.1:35120) [10]
    * transport: TCPROS
 * topic: /tf
    * to: /location (http://localhost:45309/)
    * direction: inbound (43412 - localhost:33041) [14]
    * transport: TCPROS
 * topic: /tf
    * to: /isward/camera (http://localhost:41425/)
    * direction: inbound (45566 - localhost:43167) [17]
    * transport: TCPROS
 * topic: /tf
    * to: /scheduler (http://localhost:39595/)
    * direction: inbound (34936 - localhost:38383) [18]
    * transport: TCPROS
 * topic: /tf
    * to: /charger_location_cpp (http://localhost:44709/)
    * direction: inbound (56356 - localhost:57353) [19]
    * transport: TCPROS
 * topic: /tf_static
    * to: /base_link_to_isward_link (http://localhost:36121/)
    * direction: inbound (35424 - localhost:43569) [13]
    * transport: TCPROS
 * topic: /tf_static
    * to: /isward/camera (http://localhost:41425/)
    * direction: inbound (45554 - localhost:43167) [16]
    * transport: TCPROS


=== /rosout ===
--------------------------------------------------------------------------------
Node [/rosout]
Publications: 
 * /rosout_agg [rosgraph_msgs/Log]

Subscriptions: 
 * /rosout [rosgraph_msgs/Log]

Services: 
 * /rosout/get_loggers
 * /rosout/set_logger_level


contacting node http://localhost:33751/ ...
Pid: 5176
Connections:
 * topic: /rosout
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (53870 - localhost:59801) [16]
    * transport: TCPROS
 * topic: /rosout
    * to: /hal_remote (http://localhost:43857/)
    * direction: inbound (39692 - localhost:39875) [17]
    * transport: TCPROS
 * topic: /rosout
    * to: /hal_ble (http://localhost:43287/)
    * direction: inbound (37194 - localhost:39447) [18]
    * transport: TCPROS
 * topic: /rosout
    * to: /hal_gnss (http://localhost:40065/)
    * direction: inbound (39504 - localhost:58917) [12]
    * transport: TCPROS
 * topic: /rosout
    * to: /hal_rtcm (http://localhost:34571/)
    * direction: inbound (46838 - localhost:60135) [15]
    * transport: TCPROS
 * topic: /rosout
    * to: /hal_usim (http://localhost:35735/)
    * direction: inbound (50916 - localhost:47283) [22]
    * transport: TCPROS
 * topic: /rosout
    * to: /location (http://localhost:45309/)
    * direction: inbound (43388 - localhost:33041) [23]
    * transport: TCPROS
 * topic: /rosout
    * to: /parking (http://localhost:45289/)
    * direction: inbound (47240 - localhost:46063) [24]
    * transport: TCPROS
 * topic: /rosout
    * to: /scheduler (http://localhost:39595/)
    * direction: inbound (34860 - localhost:38383) [25]
    * transport: TCPROS
 * topic: /rosout
    * to: /switcher (http://localhost:45489/)
    * direction: inbound (51470 - localhost:43583) [21]
    * transport: TCPROS
 * topic: /rosout
    * to: /base_link_to_isward_link (http://localhost:36121/)
    * direction: inbound (35390 - localhost:43569) [28]
    * transport: TCPROS
 * topic: /rosout
    * to: /mapping_node (http://localhost:45409/)
    * direction: inbound (41616 - localhost:53825) [29]
    * transport: TCPROS
 * topic: /rosout
    * to: /move_base (http://localhost:42511/)
    * direction: inbound (56940 - localhost:59579) [13]
    * transport: TCPROS
 * topic: /rosout
    * to: /pointcloud_to_laserscan (http://localhost:46083/)
    * direction: inbound (35120 - localhost:56397) [20]
    * transport: TCPROS
 * topic: /rosout
    * to: /map_server (http://localhost:39719/)
    * direction: inbound (46178 - localhost:57633) [30]
    * transport: TCPROS
 * topic: /rosout
    * to: /isward/camera (http://localhost:41425/)
    * direction: inbound (45540 - localhost:43167) [26]
    * transport: TCPROS
 * topic: /rosout
    * to: /check_camera_node (http://localhost:45895/)
    * direction: inbound (37238 - localhost:42581) [27]
    * transport: TCPROS
 * topic: /rosout
    * to: /charger_location_cpp (http://localhost:44709/)
    * direction: inbound (56348 - localhost:57353) [31]
    * transport: TCPROS


=== /scheduler ===
--------------------------------------------------------------------------------
Node [/scheduler]
Publications: 
 * /chassis/set_velocity [geometry_msgs/Twist]
 * /chassis/trigger [std_msgs/UInt8MultiArray]
 * /collision [std_msgs/Bool]
 * /host_ctrl/dev_to_host [interface_pkg/ChassisMessage]
 * /keyboard_ctrl/dev_to_keyboard [interface_pkg/ChassisStream]
 * /navi/cmd/cancel [actionlib_msgs/GoalID]
 * /navi/cmd/goal [interface_pkg/NaviCtrlActionGoal]
 * /remote_ctrl/dev_to_remote [interface_pkg/ChassisMessage]
 * /rosout [rosgraph_msgs/Log]
 * /scheduler/safe_with [std_msgs/Float32]
 * /scheduler/state [std_msgs/String]
 * /switch_location [std_msgs/Bool]
 * /test/rslt [std_msgs/String]
 * /tf [tf2_msgs/TFMessage]

Subscriptions: 
 * /chassis/batteries [interface_pkg/ChassisBatteries]
 * /chassis/fault [interface_pkg/ChassisFault]
 * /chassis/gnss [sensor_msgs/NavSatFix]
 * /chassis/imu [sensor_msgs/Imu]
 * /chassis/odometer [interface_pkg/ChassisOdometer]
 * /chassis/odometer_total [interface_pkg/ChassisOdometer]
 * /chassis/rtcm_stream [std_msgs/UInt8MultiArray]
 * /chassis/sensor [interface_pkg/ChassisSensor]
 * /cmd_vel [geometry_msgs/Twist]
 * /exps/obstacle_signal [unknown type]
 * /host_ctrl/host_to_dev [interface_pkg/ChassisMessage]
 * /keyboard_ctrl/keyboard_to_dev [interface_pkg/ChassisStream]
 * /location_state [std_msgs/UInt8MultiArray]
 * /nav_node_init [std_msgs/Bool]
 * /navi/cmd/feedback [unknown type]
 * /navi/cmd/result [unknown type]
 * /navi/cmd/status [unknown type]
 * /navi/plan_path [unknown type]
 * /parking_vision_init [std_msgs/Bool]
 * /remote/iot [std_msgs/Bool]
 * /remote_ctrl/remote_to_dev [interface_pkg/ChassisMessage]
 * /tf [tf2_msgs/TFMessage]
 * /tf_static [tf2_msgs/TFMessage]
 * /usim/rssi [std_msgs/Byte]

Services: 
 * /get_model
 * /get_version
 * /scheduler/get_loggers
 * /scheduler/set_logger_level
 * /test


contacting node http://localhost:39595/ ...
Pid: 5283
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (38383 - 127.0.0.1:34860) [11]
    * transport: TCPROS
 * topic: /switch_location
    * to: /location
    * direction: outbound (38383 - 127.0.0.1:34880) [16]
    * transport: TCPROS
 * topic: /collision
    * to: /move_base
    * direction: outbound (38383 - 127.0.0.1:34894) [9]
    * transport: TCPROS
 * topic: /chassis/set_velocity
    * to: /hal_chassis
    * direction: outbound (38383 - 127.0.0.1:34898) [22]
    * transport: TCPROS
 * topic: /host_ctrl/dev_to_host
    * to: /hal_ble
    * direction: outbound (38383 - 127.0.0.1:34896) [30]
    * transport: TCPROS
 * topic: /remote_ctrl/dev_to_remote
    * to: /hal_remote
    * direction: outbound (38383 - 127.0.0.1:34910) [29]
    * transport: TCPROS
 * topic: /keyboard_ctrl/dev_to_keyboard
    * to: /hal_chassis
    * direction: outbound (38383 - 127.0.0.1:34916) [36]
    * transport: TCPROS
 * topic: /tf
    * to: /scheduler
    * direction: outbound
    * transport: INTRAPROCESS
 * topic: /tf
    * to: /move_base
    * direction: outbound (38383 - 127.0.0.1:34932) [32]
    * transport: TCPROS
 * topic: /tf
    * to: /pointcloud_to_laserscan
    * direction: outbound (38383 - 127.0.0.1:34936) [33]
    * transport: TCPROS
 * topic: /tf
    * to: /location (http://localhost:45309/)
    * direction: inbound (43416 - localhost:33041) [18]
    * transport: TCPROS
 * topic: /tf
    * to: /isward/camera (http://localhost:41425/)
    * direction: inbound (45582 - localhost:43167) [17]
    * transport: TCPROS
 * topic: /tf
    * to: /scheduler (http://localhost:39595/)
    * direction: inbound
    * transport: INTRAPROCESS
 * topic: /tf
    * to: /charger_location_cpp (http://localhost:44709/)
    * direction: inbound (56388 - localhost:57353) [47]
    * transport: TCPROS
 * topic: /tf_static
    * to: /base_link_to_isward_link (http://localhost:36121/)
    * direction: inbound (35412 - localhost:43569) [12]
    * transport: TCPROS
 * topic: /tf_static
    * to: /isward/camera (http://localhost:41425/)
    * direction: inbound (45550 - localhost:43167) [23]
    * transport: TCPROS
 * topic: /chassis/gnss
    * to: /hal_gnss (http://localhost:40065/)
    * direction: inbound (39518 - localhost:58917) [13]
    * transport: TCPROS
 * topic: /usim/rssi
    * to: /hal_usim (http://localhost:35735/)
    * direction: inbound (50932 - localhost:47283) [14]
    * transport: TCPROS
 * topic: /remote/iot
    * to: /hal_remote (http://localhost:43857/)
    * direction: inbound (39694 - localhost:39875) [15]
    * transport: TCPROS
 * topic: /location_state
    * to: /location (http://localhost:45309/)
    * direction: inbound (43422 - localhost:33041) [19]
    * transport: TCPROS
 * topic: /nav_node_init
    * to: /move_base (http://localhost:42511/)
    * direction: inbound (56952 - localhost:59579) [20]
    * transport: TCPROS
 * topic: /parking_vision_init
    * to: /charger_location_cpp (http://localhost:44709/)
    * direction: inbound (56384 - localhost:57353) [46]
    * transport: TCPROS
 * topic: /chassis/sensor
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (53968 - localhost:59801) [25]
    * transport: TCPROS
 * topic: /chassis/odometer_total
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (53982 - localhost:59801) [26]
    * transport: TCPROS
 * topic: /chassis/odometer
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (53984 - localhost:59801) [27]
    * transport: TCPROS
 * topic: /chassis/imu
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (53986 - localhost:59801) [28]
    * transport: TCPROS
 * topic: /chassis/batteries
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (54000 - localhost:59801) [21]
    * transport: TCPROS
 * topic: /chassis/rtcm_stream
    * to: /hal_rtcm (http://localhost:34571/)
    * direction: inbound (46860 - localhost:60135) [37]
    * transport: TCPROS
 * topic: /chassis/fault
    * to: /hal_ble (http://localhost:43287/)
    * direction: inbound (37200 - localhost:39447) [38]
    * transport: TCPROS
 * topic: /chassis/fault
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (54010 - localhost:59801) [35]
    * transport: TCPROS
 * topic: /chassis/fault
    * to: /hal_rtcm (http://localhost:34571/)
    * direction: inbound (46870 - localhost:60135) [39]
    * transport: TCPROS
 * topic: /chassis/fault
    * to: /hal_gnss (http://localhost:40065/)
    * direction: inbound (39524 - localhost:58917) [41]
    * transport: TCPROS
 * topic: /cmd_vel
    * to: /parking (http://localhost:45289/)
    * direction: inbound (47256 - localhost:46063) [40]
    * transport: TCPROS
 * topic: /cmd_vel
    * to: /move_base (http://localhost:42511/)
    * direction: inbound (56962 - localhost:59579) [42]
    * transport: TCPROS
 * topic: /host_ctrl/host_to_dev
    * to: /hal_ble (http://localhost:43287/)
    * direction: inbound (37204 - localhost:39447) [31]
    * transport: TCPROS
 * topic: /remote_ctrl/remote_to_dev
    * to: /hal_remote (http://localhost:43857/)
    * direction: inbound (39708 - localhost:39875) [24]
    * transport: TCPROS
 * topic: /keyboard_ctrl/keyboard_to_dev
    * to: /hal_chassis (http://localhost:39453/)
    * direction: inbound (54018 - localhost:59801) [34]
    * transport: TCPROS


=== /switcher ===
--------------------------------------------------------------------------------
Node [/switcher]
Publications: 
 * /rosout [rosgraph_msgs/Log]

Subscriptions: None

Services: 
 * /switcher/cmd
 * /switcher/get_loggers
 * /switcher/set_logger_level


contacting node http://localhost:45489/ ...
Pid: 5301
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound (43583 - 127.0.0.1:51470) [10]
    * transport: TCPROS
