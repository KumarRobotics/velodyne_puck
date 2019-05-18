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

#include <velodyne_puck_msgs/VelodynePacket.h>

namespace velodyne_puck_driver {

using velodyne_puck_msgs::VelodynePacket;

/**
 * @brief The VelodynePuckDriver class
 */
class VelodynePuckDriver {
 public:
  VelodynePuckDriver(const ros::NodeHandle& n, const ros::NodeHandle& pn);
  ~VelodynePuckDriver();

  using Ptr = boost::shared_ptr<VelodynePuckDriver>;
  using ConstPtr = boost::shared_ptr<const VelodynePuckDriver>;

  bool Poll();

 private:
  bool OpenUdpPort();
  int ReadPacket(VelodynePacket& packet) const;

  // Ethernet relate variables
  std::string device_ip_str;
  in_addr device_ip;
  int socket_id{-1};

  // ROS related variables
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  ros::Publisher packet_pub;

  // Diagnostics updater
  diagnostic_updater::Updater diagnostics;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
  double diag_min_freq;
  double diag_max_freq;
};

}  // namespace velodyne_puck_driver
