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
  /// ==========================================================================
  /// 9.3.1.3 Data Point
  /// A data point is a measurement by one laser channel of a relection of a
  /// laser pulse
  struct DataPoint {
    uint16_t distance;
    uint8_t reflectivity;
  } __attribute__((packed));
  static_assert(sizeof(DataPoint) == 3, "sizeof(DataPoint) != 3");

  /// 9.3.1.1 Firing Sequence
  /// A firing sequence occurs when all the lasers in a sensor are fired. There
  /// are 16 firings per cycle for VLP-16
  struct FiringSequence {
    DataPoint points[kFiringsPerSequence];  // 16
  } __attribute__((packed));
  static_assert(sizeof(FiringSequence) == 3 * 16,
                "sizeof(FiringSequence) != 48");

  /// 9.3.1.4 Azimuth
  /// A two-bytle azimuth value (alpha) appears after the flag bytes at the
  /// beginning of each data block
  ///
  /// 9.3.1.5 Data Block
  /// The information from 2 firing sequences of 16 lasers is contained in each
  /// data block. Each packet contains the data from 24 firing sequences in 12
  /// data blocks.
  ///
  struct DataBlock {
    uint16_t flag;
    uint16_t azimuth;
    FiringSequence sequences[kSequencePerBlock];  // 2
  } __attribute__((packed));
  static_assert(sizeof(DataBlock) == 100, "sizeof(DataBlock) != 100");

  struct Packet {
    DataBlock blocks[kBlocksPerPacket];  // 12
    /// The four-byte time stamp is a 32-bit unsigned integer marking the moment
    /// of the first data point in the first firing sequcne of the first data
    /// block
    uint32_t stamp;
    uint8_t factory[2];
  } __attribute__((packed));
  static_assert(sizeof(Packet) == 1206, "sizeof(Packet) != 1206");

  struct Decoded {
    struct Firing {
      float azimuth;
      float distance[kFiringsPerSequence];  // 16
      float reflectivity[kFiringsPerSequence];
    };

    Firing firings[kFiringsPerPacket];  // 2 * 12 = 24
  };
  /// ==========================================================================

  union TwoBytes {
    uint16_t distance;
    uint8_t bytes[2];
  };

  struct RawBlock {
    /// The information from 2 firing sequences of 16 lasers is contained in
    /// each data block
    uint16_t flag;     /// UPPER_BANK or LOWER_BANK, 2 byte flag
    uint16_t azimuth;  /// 0-35999, divide by 100 to get degrees
    uint8_t data[kPointBytesPerBlock];  /// 96
  };

  static_assert(sizeof(RawBlock) == 100, "DataBlock size must be 100");
  struct RawPacket {
    RawBlock blocks[kBlocksPerPacket];  // 12
    /// The four-byte time stamp is a 32-bit unsigned integer marking the moment
    /// of the first data point in the first firing sequcne of the first data
    /// block
    uint32_t stamp;
    uint8_t factory[2];
  } __attribute__((packed));
  static_assert(sizeof(RawPacket) == 1206, "sizeof(RawPacket) != 1206");

  struct Firing {
    // Azimuth associated with the first shot within this firing.
    float firing_azimuth;
    float azimuth[kFiringsPerSequence];
    float distance[kFiringsPerSequence];
    float intensity[kFiringsPerSequence];
  };

  // Callback function for a single velodyne packet.
  bool checkPacketValidity(const RawPacket* packet);
  Decoded DecodePacket(const RawPacket* packet);
  void PacketCb(const VelodynePacketConstPtr& packet_msg);

  // Publish data
  void PublishCloud(const VelodyneSweep& sweep_msg);

  // Check if a point is in the required range.
  bool IsPointInRange(float distance) const {
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
