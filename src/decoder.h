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

#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>
#include <velodyne_puck/VelodynePacket.h>

#include "constants.h"

namespace velodyne_puck {

/**
 * @brief The VelodynePuckDecoder class
 */
class Decoder {
 public:
  Decoder(const ros::NodeHandle& n, const ros::NodeHandle& pn);
  Decoder(const Decoder&) = delete;
  Decoder operator=(const Decoder&) = delete;

  using Ptr = boost::shared_ptr<Decoder>;
  using ConstPtr = boost::shared_ptr<const Decoder>;

  void PacketCb(const VelodynePacketConstPtr& packet_msg);

 private:
  /// ==========================================================================
  /// All of these uses laser index from velodyne which is interleaved
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
    DataPoint points[kFiringsPerFiringSequence];  // 16
  } __attribute__((packed));
  static_assert(sizeof(FiringSequence) == 3 * 16,
                "sizeof(FiringSequence) != 48");

  /// 9.3.1.4 Azimuth
  /// A two-byte azimuth value (alpha) appears after the flag bytes at the
  /// beginning of each data block
  ///
  /// 9.3.1.5 Data Block
  /// The information from 2 firing sequences of 16 lasers is contained in each
  /// data block. Each packet contains the data from 24 firing sequences in 12
  /// data blocks.
  struct DataBlock {
    uint16_t flag;
    uint16_t azimuth;                                        // [0, 35999]
    FiringSequence sequences[kFiringSequencesPerDataBlock];  // 2
  } __attribute__((packed));
  static_assert(sizeof(DataBlock) == 100, "sizeof(DataBlock) != 100");

  struct Packet {
    DataBlock blocks[kDataBlocksPerPacket];  // 12
    /// The four-byte time stamp is a 32-bit unsigned integer marking the moment
    /// of the first data point in the first firing sequcne of the first data
    /// block
    uint32_t stamp;
    uint8_t factory[2];
  } __attribute__((packed));
  static_assert(sizeof(Packet) == sizeof(VelodynePacket().data),
                "sizeof(Packet) != 1206");

  /// Decoded result
  struct TimedFiringSequence {
    double time;
    float azimuth;  // rad [0, 2pi)
    FiringSequence sequence;
  };

  using Decoded = std::array<TimedFiringSequence, kFiringSequencesPerPacket>;

  union TwoBytes {
    uint16_t u16;
    uint8_t u8[2];
  };

  Decoded DecodePacket(const Packet* packet, double time);

  using RangeImage =
      std::pair<sensor_msgs::ImagePtr, sensor_msgs::CameraInfoPtr>;
  /// Convert firing sequences to range image
  RangeImage ToRangeImage(const std::vector<TimedFiringSequence>& tfseqs) const;

  void PublishImage(const RangeImage& range_image);
  void PublishCloud(const RangeImage& range_image);

  // Configuration parameters
  double min_range;
  double max_range;

  // ROS related parameters
  std::string frame_id;

  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  image_transport::ImageTransport it;

  ros::Subscriber packet_sub;
  ros::Publisher sweep_pub;
  ros::Publisher cloud_pub;
  image_transport::CameraPublisher camera_pub;

  std::vector<TimedFiringSequence> buffer_;  // buffer
};

}  // namespace velodyne_puck
