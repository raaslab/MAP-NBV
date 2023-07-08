# MAP-NBV


## Run MAP-NBV

Start AirSim in Unreal Engine environment

Run AirSim ROS Wrapper in 1st terminal
```
roslaunch airsim_ros_pkgs airsim_node.launch
```

Set AirSim Semantic Segmentation Values in 2nd terminal
```
python3 set_segmentation.py
```

PoinTr to AirSim TF
```
rosrun tf2_ros static_transform_publisher 0 0 0 0.7071068 0 0 -0.7071068 world_ned pointr
```

Depth Segmentation in 3rd terminal
```
roslaunch airsim_moveit_navigation depth_segmentation.launch
```

Depth to Pointcloud in 4th terminal
```
roslaunch depth_image_proc dual_point_cloud_xyz_radial.launch
```

Merge PointClouds in 5th terminal
```
roslaunch point_cloud_processing point_cloud_merge.launch
```

Airsim Moveit in 6th terminal
```
roslaunch airsim_moveit_config multiprednbv.launch
```

Run AirSim + MoveIt Client in 7th terminal
```
rosrun airsim_moveit_navigation airsim_multiprednbv_navigator
```

Drone Movement Control in 8th terminal
```
rosrun airsim_moveit_navigation airsim_multiagent_client
```

Reconstruction Pipeline
```
rosrun airsim_moveit_navigation pointr_reconstruction_multiagent.py
```

## Run Multi-agent Baseline

Start AirSim in Unreal Engine environment

Run AirSim ROS Wrapper in 1st terminal
```
roslaunch airsim_ros_pkgs airsim_node.launch
```

Set AirSim Semantic Segmentation Values in 2nd terminal
```
python3 set_segmentation.py
```
Merge PointClouds in 3rd terminal
```
roslaunch point_cloud_processing point_cloud_merge.launch
```
OctoMap in 4th terminal
```
rosrun octomap_server octomap_server_node _resolution:=1 _frame_id:="world_enu" _sensor_model/max_range:=200 cloud_in:=merged_point_cloud
```
Depth Segmentation in 5th terminal
```
roslaunch airsim_moveit_navigation depth_segmentation.launch
```
Depth to Pointcloud in 6th terminal
```
roslaunch depth_image_proc dual_point_cloud_xyz_radial.launch
```
Occupany Grid Publisher in 7th terminal
```
rosrun octomap_server nbv_pointcloud
```
Baseline NBV Package in 8th terminal
```
roslaunch basic_next_best_view nbv.launch
```
Airsim Moveit in 9th terminal
```
roslaunch airsim_moveit_config multiprednbv.launch
```
Airsim Moveit Navigation in 10th terminal
```
rosrun airsim_moveit_navigation airsim_multiprednbv_navigator
```
Drone Movement Control in 11th terminal
```
rosrun airsim_moveit_navigation airsim_multiagent_client
```
Baseline Planner in 12th terminal
```
rosrun airsim_moveit_navigation multiagent_baseline_nbv.py
```