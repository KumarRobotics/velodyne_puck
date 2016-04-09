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

#include <velodyne_puck_driver/velodyne_puck_driver_nodelet.h>


namespace velodyne_puck_driver
{

VelodynePuckDriverNodelet::VelodynePuckDriverNodelet():
  running(false) {
  return;
}

VelodynePuckDriverNodelet::~VelodynePuckDriverNodelet() {
  if (running) {
    NODELET_INFO("shutting down driver thread");
    running = false;
    device_thread->join();
    NODELET_INFO("driver thread stopped");
  }
  return;
}

void VelodynePuckDriverNodelet::onInit()
{
  // start the driver
  velodyne_puck_driver.reset(
      new VelodynePuckDriver(getNodeHandle(), getPrivateNodeHandle()));
  if (!velodyne_puck_driver->initialize()) {
    ROS_ERROR("Cannot initialize Velodyne driver...");
    return;
  }

  // spawn device poll thread
  running = true;
  device_thread = boost::shared_ptr< boost::thread >
    (new boost::thread(boost::bind(&VelodynePuckDriverNodelet::devicePoll, this)));
}

/** @brief Device poll thread main loop. */
void VelodynePuckDriverNodelet::devicePoll()
{
  while(ros::ok()) {
    // poll device until end of file
    running = velodyne_puck_driver->polling();
    if (!running)
      break;
  }
  running = false;
}

} // namespace velodyne_driver

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_puck_driver, VelodynePuckDriverNodelet,
                        velodyne_puck_driver::VelodynePuckDriverNodelet, nodelet::Nodelet);
