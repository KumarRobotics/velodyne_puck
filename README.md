# velodyne_puck

[![Build Status](https://travis-ci.org/KumarRobotics/velodyne_puck.svg?branch=master)](https://travis-ci.org/KumarRobotics/velodyne_puck)

![Picture of Velodyne Puck](http://velodynelidar.com/images/products/vlp-16/puck.png)

The `velodyne_puck` package is a linux ROS driver for velodyne puck only of [VELODYNE LIDAR](http://velodynelidar.com/).

The user manual for the device can be found [here](http://velodynelidar.com/vlp-16.html) or the Lite version [here](http://velodynelidar.com/vlp-16-lite.html).

The major difference between this driver and the [ROS velodyne driver](http://wiki.ros.org/velodyne_driver) is that the start of each revolution is detected using azimuth. Also publish a range image.

The package is tested on Ubuntu 18.04 with ROS melodic.

## License

This driver is developed based on [ROS velodyne driver](http://wiki.ros.org/velodyne_driver), which originally has the BSD license. The COPYING file is kept in this package. However, the changed files have the GNU General Public License V3.0.

## Example Usage

### velodyne_puck_driver

**Parameters**

`device_ip` (`string`, `default: 192.168.1.201`)

By default, the IP address of the device is 192.168.1.201.

**Published Topics**

`packet` (`velodyne_puck/VelodynePacket`)

Each message corresponds to a velodyne packet sent by the device through the Ethernet. For more details on the definition of the packet, please refer to the [user manual](http://velodynelidar.com/docs/manuals/63-9243%20Rev%20B%20User%20Manual%20and%20Programming%20Guide,VLP-16.pdf).

### velodyne_puck_decoder

**Parameters**

`min_range` (`double`, `0.5`)

`max_range` (`double`, `100.0`)

Points outside this range will be removed only in published point cloud.

`organized` (`bool`, `true`)

Whether to publish an organized cloud or not. Does not affect the range image.

`frame_id` (`string`, `velodyne`)

Will be used as namespace for all nodes and messages.

**Published Topics**

`image` (`sensor_msgs/Image`)

The range image is encoded as a BGR8 image, since the data point from the device contains 3 bytes, 2 for distance and 1 for intensity.

row 0 ~ 15 in the image correspond to lasers from top to bottom, as opposed to bottom to top for visualization purposes.

pixels in each row just contain those 3 bytes.

`cinfo` (`sensor_msgs/CameraInfo`)

Stores relevant information to restore points from range image.

```
K[0] = MinElevation; // -15 deg
K[1] = MaxElevation; // 15 deg
K[2] = DistanceResolution; // 0.002
k[3] = FiringCycleUs; // 55.296 us
D = vector<azimuth> // D.size() == image.cols
```

`cloud` (`sensor_msgs/PointCloud2`)

A point cloud, where invalid points are filled with NaNs if organized and removed if not organized.

**Node**

```
roslaunch velodyne_puck run.launch driver:=true device_ip:=192.168.1.201
```
