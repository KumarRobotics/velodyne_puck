/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver nodelet for the Velodyne 3D LIDARs
 */

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <velodyne_puck_driver/velodyne_puck_driver.h>

namespace velodyne_puck_driver
{

class VelodynePuckDriverNodelet: public nodelet::Nodelet
{
public:

  VelodynePuckDriverNodelet();
  ~VelodynePuckDriverNodelet();

private:

  virtual void onInit(void);
  virtual void devicePoll(void);

  volatile bool running;               ///< device thread is running
  boost::shared_ptr<boost::thread> device_thread;

  VelodynePuckDriverPtr velodyne_puck_driver; ///< driver implementation class
};

} // namespace velodyne_driver
