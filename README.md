# velodyne_puck

[![Build Status](https://travis-ci.org/KumarRobotics/velodyne_puck.svg?branch=master)](https://travis-ci.org/KumarRobotics/velodyne_puck)

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
This is the topic to save, don't save point cloud or range image!!!

### velodyne_puck_decoder

**Parameters**

`min_range` (`double`, `0.5`)

`max_range` (`double`, `100.0`)

Points outside this range will be NaN in published point cloud if `organized=True`.
Otherwise, they will be removed.

`image_width` (`int`, `1024`)

Width of the published image.

`full_sweep` (`bool`, `false`)

Whether to publish a full sweep or not.

`organized` (`bool`, `true`)

Whether to publish an organized cloud or not. 

`frame_id` (`string`, `velodyne`)

Will be used as namespace for all nodes and messages.

**Published Topics**

`image` (`sensor_msgs/Image`)

The range image is encoded as a CV_F32C3 image, with channels (Range, Intensity, Azimuth)
row 0 ~ 15 in the image correspond to lasers from top (15deg) to bottom (-15deg).

pixels in each row just contain those 3 bytes.

`cinfo` (`sensor_msgs/CameraInfo`)

Stores relevant information to restore points from range image.

`intensity` (`sensor_msgs/Image`)

For visualizing intensity image only.

`range` (`sensor_msgs/Image`)

For visualizing range image only.


```
P[0] = FiringCycleNs; // 55296 ns
D = [elevations, azimuths] // D.size() == image.height + image.width
```

`cloud` (`sensor_msgs/PointCloud2`)

A point cloud, where invalid points are filled with NaNs if organized and removed if not organized.

**Node**

Run full driver
```
roslaunch velodyne_puck run.launch driver:=true device_ip:=192.168.1.201
```

Run decoder only
```
roslaunch velodyne_puck run.launch driver:=false
```
