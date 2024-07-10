# Livox MID-360 ROS Driver2 for `Humble` only

The `Livox-SDK2` should be installed first: [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md)
Compared with the original package, `build.sh` is deleted so only `humble` is supported. Now the package needn't be compiled independently.

For the original package info and more information about config file, see:

[Official livox-ros2-driver git repo](https://github.com/Livox-SDK/livox_ros_driver2)

[User Manual and Other documents](https://www.livoxtech.com/mid-360/downloads1)

# Trouble Shooting

1. Keep the host ip address consistent with the config file of `MID360_config.json` host address.
2. Change the lidar ip address in `MID360_config.json` to the address of the lidar address, which can be got from the last two digits of serial number.
3. Livox communication is based on UDP, so the routing tables may be checked if no data received.
4. Official software [Livox Viewer 2](https://www.livoxtech.com/de/downloads) stills works for Ubuntu22.04, which can be used to check whether lidar is in function.
5. To see the laserscan data in rviz, the reliablity attribute should be changed in rviz.


