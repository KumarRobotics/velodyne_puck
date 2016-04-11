/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver node for the Velodyne 3D LIDARs.
 */

#include <ros/ros.h>
#include <velodyne_puck_driver/velodyne_puck_driver.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velodyne_puck_driver_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // start the driver
  velodyne_puck_driver::VelodynePuckDriver driver(node, private_nh);

  // loop until shut down or end of file
  while(ros::ok() && driver.polling()) {
    ros::spinOnce();
  }

  return 0;
}
