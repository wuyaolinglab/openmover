# lslidar

## Description

The `lslidar` package is a linux ROS driver for lslidar M10 ,L10,M10_GPS,M10_P,M10_PLUS and N10.
The package is tested on Ubuntu 20.04 with ROS indigo.

## Compling

This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
catkin_make 
source devel/setup.bash
roslaunch lslidar_driver lslidar_net.launch 				(网口版雷达)
roslaunch lslidar_driver lslidar_serial.launch 				(串口版雷达)
```

open new terminal
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 1"		(open radar)
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 0"		(close radar)

```
open new terminal
source devel/setup.bash 
rostopic echo /difop_information					(printf information)
## Example Usage

### lslidar_m10_driver

**Parameters**

`device_ip` (`string`, `default: 192.168.1.222`)

By default, the IP address of the device is 192.168.1.222.

`frame_id` (`string`, `default: lslidar`)

The frame ID entry for the sent messages.

**Published Topics**

`lslidar_packets` (`lslidar_m10_msgs/LslidarM10Packet`)

Each message corresponds to a lslidar packet sent by the device through the Ethernet.

### lslidar_m10_decoder

**Parameters**

`min_range` (`double`, `0.3`)

`max_range` (`double`, `100.0`)

Points outside this range will be removed.

`frequency` (`frequency`, `20.0`)

Note that the driver does not change the frequency of the sensor. 

`publish_point_cloud` (`bool`, `false`)

If set to true, the decoder will additionally send out a local point cloud consisting of the points in each revolution.

**Published Topics**

`lslidar_sweep` (`lslidar_m10_msgs/LslidarM10Sweep`)

The message arranges the points within each sweep based on its scan index and azimuth.

`lslidar_point_cloud` (`sensor_msgs/PointCloud2`)

This is only published when the `publish_point_cloud` is set to `true` in the launch file.

**Node**

```

roslaunch lslidar_driver lslidar_net.launch

```

Note that this launch file launches both the driver and the decoder, which is the only launch file needed to be used.


## FAQ


## Bug Report

Prefer to open an issue. You can also send an E-mail to shaohuashu@lslidar.com




RERTION 
V0.1 2000 points per circle
v0.2 1000 points per circle

```
