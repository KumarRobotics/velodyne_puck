/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include <algorithm>
#include "convert.h"

#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_msgs/VelodynePoint.h>
#include <velodyne_msgs/VelodynePointScan.h>
#include <velodyne_msgs/VelodynePointCloudByScan.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);


    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<velodyne_msgs::VelodynePointCloudByScan>("velodyne_points", 10);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      VelodyneConfigConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::VelodyneConfigConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }

  bool velodynePointCompare(const velodyne_msgs::VelodynePoint& p1,
      const velodyne_msgs::VelodynePoint& p2) {
    return p1.azimuth < p2.azimuth;
  }

  void Convert::callback(velodyne_pointcloud::VelodyneConfigConfig &config,
                uint32_t level)
  {
    ROS_INFO("Reconfigure Request");
    data_->setParameters(config.min_range, config.max_range, config.view_direction,
                         config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    velodyne_msgs::VelodynePointCloudByScanPtr outMsg(
        new velodyne_msgs::VelodynePointCloudByScan());
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg->header.stamp = scanMsg->header.stamp;
    outMsg->header.frame_id = scanMsg->header.frame_id;

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      //printf("block start time: %f\n", i*velodyne_rawdata::VLP16_BLOCK_TDURATION*12);
      data_->setPacketStartTime(i*velodyne_rawdata::VLP16_BLOCK_TDURATION*12);
      data_->unpack(scanMsg->packets[i], *outMsg);
      //printf("\n");
    }

    // Sort the point in each scan based on the azimuth
    for (size_t i = 0; i < 16; ++i) {
      outMsg->scans[i].ring = outMsg->scans[i].points[0].ring;
      outMsg->scans[i].altitude = outMsg->scans[i].points[0].altitude;
      std::sort(outMsg->scans[i].points.begin(),
          outMsg->scans[i].points.end(), velodynePointCompare);
    }

    // publish the accumulated cloud message
    output_.publish(outMsg);
  }

} // namespace velodyne_pointcloud
