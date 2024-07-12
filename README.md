# Fast-LIO-Tinker

Fast-LIO2 For Tinker is based on [Ericsii / FAST_LIO_ROS](https://github.com/Ericsii/FAST_LIO_ROS2), which is an implementation of fast-lio2 on Livox MID 360.
It uses Fast-LIO2(3D SLAM) to build the grid map.
This repository relies on [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2).

Unless specially mentioned, all the following instructions should be executed in this repository.

## Launch Fast-LIO2

```shell
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

## Launch Chassis & Lidar

In repository [tk23_navigation](https://github.com/tinkerfuroc/tk23_navigation):

```shell
sudo ./setupcan.sh
./chassis_lidar_bringup.zsh
```

## Run Lidar Odom

The `lidar_odom` here only publishes filtered pointcloud2. It **does not** publish odom.

```powershell
ros2 run lidar_odom lidar_odom
```

## Launch Scan Convert

Convert `/Laser_map` (which is in pointcloud2) to `/scan`

```shell
ros2 launch livox_ros_driver2 convert_launch.py
```
