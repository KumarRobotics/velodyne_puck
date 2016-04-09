/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the Velodyne 3D LIDARs
 */

#ifndef VELODYNE_PUCK_DRIVER_H
#define VELODYNE_PUCK_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <velodyne_puck_msgs/VelodynePacket.h>

namespace velodyne_puck_driver {

static uint16_t UDP_PORT_NUMBER = 2368;
static uint16_t PACKET_SIZE = 1206;

class VelodynePuckDriver {
public:

  VelodynePuckDriver(ros::NodeHandle& n, ros::NodeHandle& pn);
  ~VelodynePuckDriver();

  bool initialize();
  bool polling();

  typedef boost::shared_ptr<VelodynePuckDriver> VelodynePuckDriverPtr;
  typedef boost::shared_ptr<const VelodynePuckDriver> VelodynePuckDriverConstPtr;

private:

  bool loadParameters();
  bool createRosIO();
  bool openUDPPort();
  int getPacket(velodyne_puck_msgs::VelodynePacketPtr& msg);

  // Ethernet relate variables
  std::string device_ip_string;
  in_addr device_ip;
  int socket_id;

  // ROS related variables
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  std::string frame_id;
  ros::Publisher packet_pub;

  // Diagnostics updater
  diagnostic_updater::Updater diagnostics;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
  double diag_min_freq;
  double diag_max_freq;
};

typedef VelodynePuckDriver::VelodynePuckDriverPtr VelodynePuckDriverPtr;
typedef VelodynePuckDriver::VelodynePuckDriverConstPtr VelodynePuckDriverConstPtr;

} // namespace velodyne_driver

#endif // _VELODYNE_DRIVER_H_
