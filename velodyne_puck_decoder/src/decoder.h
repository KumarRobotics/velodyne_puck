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

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <velodyne_puck_msgs/VelodynePacket.h>
#include <velodyne_puck_msgs/VelodyneSweep.h>

#include "constants.h"

namespace velodyne_puck_decoder {

using velodyne_puck_msgs::VelodynePacketConstPtr;
using velodyne_puck_msgs::VelodyneSweep;

/**
 * @brief The VelodynePuckDecoder class
 */
class VelodynePuckDecoder {
 public:
  VelodynePuckDecoder(const ros::NodeHandle& n, const ros::NodeHandle& pn);
  VelodynePuckDecoder(const VelodynePuckDecoder&) = delete;
  VelodynePuckDecoder operator=(const VelodynePuckDecoder&) = delete;

  using Ptr = boost::shared_ptr<VelodynePuckDecoder>;
  using ConstPtr = boost::shared_ptr<const VelodynePuckDecoder>;

  bool Initialize();

 private:
  union TwoBytes {
    uint16_t distance;
    uint8_t bytes[2];
  };

  struct DataBlock {
    /// The information from 2 firing sequences of 16 lasers is contained in
    /// each data block
    uint16_t flag;     /// UPPER_BANK or LOWER_BANK, 2 byte flag
    uint16_t azimuth;  /// 0-35999, divide by 100 to get degrees
    uint8_t data[kPointBytesPerBlock];  /// 96
  };

  static_assert(sizeof(DataBlock) == 100, "DataBlock size must be 100");

  struct Packet {
    DataBlock blocks[kBlocksPerPacket];
    /// The four-byte time stamp is a 32-bit unsigned integer marking the moment
    /// of the first data point in the first firing sequcne of the first data
    /// block
    uint32_t stamp;
    uint8_t factory[2];
  };

  struct Firing {
    // Azimuth associated with the first shot within this firing.
    float firing_azimuth;
    float azimuth[kFiringsPerCycle];
    float distance[kFiringsPerCycle];
    float intensity[kFiringsPerCycle];
  };

  // Intialization sequence
  bool loadParameters();
  bool createRosIO();

  // Callback function for a single velodyne packet.
  bool checkPacketValidity(const Packet* packet);
  void DecodePacket(const Packet* packet);
  void PacketCb(const VelodynePacketConstPtr& packet_msg);

  // Publish data
  void PublishCloud(const VelodyneSweep& sweep_msg);

  // Check if a point is in the required range.
  bool isPointInRange(float distance) const {
    return distance >= min_range && distance <= max_range;
  }

  // Configuration parameters
  double min_range;
  double max_range;

  bool is_first_sweep{true};
  float last_azimuth{0.0};

  double sweep_start_time{0.0};
  double packet_start_time{0.0};

  Firing firings[kFiringsPerPacket];

  // ROS related parameters
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  std::string frame_id;

  ros::Subscriber packet_sub;
  ros::Publisher sweep_pub;
  ros::Publisher cloud_pub;

  velodyne_puck_msgs::VelodyneSweepPtr sweep_data;

  sensor_msgs::ImagePtr image_;
  sensor_msgs::CameraInfoPtr cinfo_;
};

}  // namespace velodyne_puck_decoder
