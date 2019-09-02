/*
 * This file is part of velodyne_puck driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

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
  bool PollPacket();
  bool PollScan();

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
