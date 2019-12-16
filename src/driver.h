#pragma once

#include <netinet/in.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <ros/ros.h>
#include <velodyne_msgs/VelodynePacket.h>

namespace velodyne_puck {

class Driver {
 public:
  explicit Driver(const ros::NodeHandle& pnh);
  ~Driver();

  using Ptr = boost::shared_ptr<Driver>;

  bool Poll();

 private:
  bool OpenUdpPort();
  int ReadPacket(velodyne_msgs::VelodynePacket& packet) const;

  // Ethernet relate variables
  std::string device_ip_str_;
  in_addr device_ip_;
  int socket_id_{-1};

  // ROS related variables
  ros::NodeHandle pnh_;
  ros::Publisher pub_packet_;
  ros::Publisher pub_scan_;

  // Diagnostics updater
  diagnostic_updater::Updater updater_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> topic_diag_;
  std::vector<velodyne_msgs::VelodynePacket> buffer_;
  double freq_;
  int batch_size_{0};
};

}  // namespace velodyne_puck
