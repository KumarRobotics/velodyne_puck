#include <pluginlib/class_list_macros.h>

#include "driver_nodelet.h"

namespace velodyne_puck {

DriverNodelet::DriverNodelet() : running(false) {}

DriverNodelet::~DriverNodelet() {
  if (running) {
    NODELET_INFO("shutting down driver thread");
    running = false;
    device_thread->join();
    NODELET_INFO("driver thread stopped");
  }
}

void DriverNodelet::onInit() {
  // start the driver
  velodyne_puck_driver.reset(new Driver(getPrivateNodeHandle()));

  // spawn device poll thread
  running = true;
  device_thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&DriverNodelet::devicePoll, this)));
}

/** @brief Device poll thread main loop. */
void DriverNodelet::devicePoll() {
  while (ros::ok()) {
    // poll device until end of file
    running = velodyne_puck_driver->Poll();
    if (!running) break;
  }
  running = false;
}

}  // namespace velodyne_puck

// parameters are: package, class name, class type, base class type
PLUGINLIB_EXPORT_CLASS(velodyne_puck::DriverNodelet, nodelet::Nodelet);
