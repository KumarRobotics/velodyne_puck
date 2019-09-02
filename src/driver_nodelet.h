#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include "driver.h"

namespace velodyne_puck {

class DriverNodelet : public nodelet::Nodelet {
 public:
  DriverNodelet();
  ~DriverNodelet();

 private:
  virtual void onInit();
  virtual void devicePoll();

  volatile bool running;  ///< device thread is running
  boost::shared_ptr<boost::thread> device_thread;

  Driver::Ptr velodyne_puck_driver;  ///< driver implementation class
};

}  // namespace velodyne_puck
