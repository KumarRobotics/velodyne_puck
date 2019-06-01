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

#include "decoder.h"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace velodyne_puck {

enum Index {
  MIN_ELEVATION,
  MAX_ELEVATION,
  DISTANCE_RESOLUTION,
  FIRING_CYCLE_US
};

union TwoBytes {
  uint16_t u16;
  uint8_t u8[2];
};

Decoder::Decoder(const ros::NodeHandle& n, const ros::NodeHandle& pn)
    : nh_(n), pnh_(pn), it_(pn) {
  pnh_.param("min_range", min_range_, 0.5);
  pnh_.param("max_range", max_range_, 100.0);
  ROS_ASSERT_MSG(min_range_ <= max_range_, "min_range > max_range");
  ROS_INFO("min_range: %f, max_range: %f", min_range_, max_range_);

  pnh_.param("organized", organized_, true);
  ROS_INFO("publish organized cloud: %s", organized_ ? "true" : "false");

  pnh_.param<std::string>("frame_id", frame_id_, "velodyne");
  ROS_INFO("Velodyne frame_id: %s", frame_id_.c_str());

  packet_sub_ =
      pnh_.subscribe<VelodynePacket>("packet", 100, &Decoder::PacketCb, this);
  cloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("cloud", 10);
  camera_pub_ = it_.advertiseCamera("image", 10);
}

Decoder::Decoded Decoder::DecodePacket(const Packet* packet,
                                       double time) const {
  // Azimuth is clockwise, which is absurd
  // ^ y
  // | a /
  // |  /
  // | /
  // |/
  // o ------- > x

  // Check return mode and product id, for now just die
  const auto return_mode = packet->factory[0];
  if (!(return_mode == 55 || return_mode == 56)) {
    ROS_ERROR(
        "return mode must be Strongest (55) or Last Return (56), "
        "instead got (%u)",
        return_mode);
    ros::shutdown();
  }
  const auto product_id = packet->factory[1];
  if (product_id != 34) {
    ROS_ERROR("product id must be VLP-16 or Puck Lite (34), instead got (%u)",
              product_id);
    ros::shutdown();
  }

  // std::array<TimedFiringSequence, kFiringSequencesPerPacket>;
  Decoded decoded;
  // For each data block, 12 total
  for (int bi = 0; bi < kDataBlocksPerPacket; ++bi) {
    const auto& block = packet->blocks[bi];
    const auto raw_azimuth = block.azimuth;
    ROS_WARN_STREAM_COND(raw_azimuth > kMaxRawAzimuth,
                         "Invalid raw azimuth: " << raw_azimuth);
    ROS_WARN_COND(block.flag != UPPER_BANK, "Invalid block %d", bi);

    // Fill in decoded
    // for each firing sequence in the data block, 2
    for (int fsi = 0; fsi < kFiringSequencesPerDataBlock; ++fsi) {
      // Index into decoded, hardcode 2 for now
      const auto di = bi * 2 + fsi;
      auto& tfseq = decoded[di];
      // Assume all firings within each firing sequence occur at the same time
      tfseq.time = time + di * kFiringCycleUs * 1e-6;
      tfseq.azimuth = Azimuth(block.azimuth);  // need to fix half later
      tfseq.sequence = block.sequences[fsi];
    }
  }

  // Fix azimuth for odd firing sequences
  for (int bi = 0; bi < kDataBlocksPerPacket; ++bi) {
    // 1,3,5,...,23
    // Index into decoded, hardcode 2 for now
    const auto di = bi * 2 + 1;
    auto azimuth = decoded[di].azimuth;

    auto prev = di - 1;
    auto next = di + 1;
    // Handle last block where there's no next to inerpolate
    // Just use the previous two
    if (bi == kDataBlocksPerPacket - 1) {
      prev -= 2;
      next -= 2;
    }

    auto azimuth_prev = decoded[prev].azimuth;
    auto azimuth_next = decoded[next].azimuth;

    // Handle angle warping
    // Based on the fact that all raw azimuth is within 0 to 2pi
    if (azimuth_next < azimuth_prev) {
      azimuth_next += kTau;
    }

    ROS_WARN_COND(azimuth_prev > azimuth_next,
                  "azimuth_prev %f > azimuth_next %f", azimuth_prev,
                  azimuth_next);

    const auto azimuth_diff = (azimuth_next - azimuth_prev) / 2.0;
    const auto azimuth_diff_deg = rad2deg(azimuth_diff);

    ROS_WARN_COND(std::abs(azimuth_diff_deg - 0.1) > 1e-1,
                  "azimuth_diff too big: %f deg", azimuth_diff_deg);

    azimuth += azimuth_diff;
    if (azimuth > kTau) {
      azimuth -= kTau;
    }

    decoded[di].azimuth = azimuth;
  }

  return decoded;
}

void Decoder::PacketCb(const VelodynePacketConstPtr& packet_msg) {
  const auto* my_packet =
      reinterpret_cast<const Packet*>(&(packet_msg->data[0]));

  // TODO: maybe make this a vector? to handle invalid data block?
  const auto decoded = DecodePacket(my_packet, packet_msg->stamp.toSec());

  // Check for a change of azimuth across 0
  float prev_azimuth = buffer_.empty() ? -1 : buffer_.back().azimuth;

  for (const auto& tfseq : decoded) {
    if (tfseq.azimuth < prev_azimuth) {
      // this indicates we cross the 0 azimuth angle
      // we are ready to publish what's in the buffer
      ROS_DEBUG("curr_azimuth: %f < %f prev_azimuth", rad2deg(tfseq.azimuth),
                rad2deg(prev_azimuth));
      ROS_DEBUG("buffer size: %zu", buffer_.size());
      PublishBufferAndClear();
    }

    buffer_.push_back(tfseq);
    prev_azimuth = tfseq.azimuth;
  }
}

void Decoder::PublishBufferAndClear() {
  if (buffer_.empty()) return;

  if (camera_pub_.getNumSubscribers() || cloud_pub_.getNumSubscribers()) {
    sensor_msgs::CameraInfoPtr cinfo_msg(new sensor_msgs::CameraInfo);
    auto image_msg = ToRangeImage(buffer_, *cinfo_msg);

    if (camera_pub_.getNumSubscribers()) {
      camera_pub_.publish(image_msg, cinfo_msg);
    }

    if (cloud_pub_.getNumSubscribers()) {
      PublishCloud(image_msg, cinfo_msg);
    }
  }

  buffer_.clear();
}

sensor_msgs::ImagePtr Decoder::ToRangeImage(
    const std::vector<TimedFiringSequence>& tfseqs,
    sensor_msgs::CameraInfo& cinfo_msg) const {
  std_msgs::Header header;
  header.stamp = ros::Time(tfseqs[0].time);
  header.frame_id = frame_id_;

  cv::Mat image =
      cv::Mat::zeros(kFiringsPerFiringSequence, tfseqs.size(), CV_8UC3);
  ROS_DEBUG("image size: %d x %d", image.rows, image.cols);

  //  sensor_msgs::CameraInfoPtr cinfo_msg(new sensor_msgs::CameraInfo);
  cinfo_msg.header = header;
  cinfo_msg.height = image.rows;
  cinfo_msg.width = image.cols;
  cinfo_msg.K[Index::MIN_ELEVATION] = kMinElevation;
  cinfo_msg.K[Index::MAX_ELEVATION] = kMaxElevation;
  cinfo_msg.K[Index::DISTANCE_RESOLUTION] = kDistanceResolution;
  cinfo_msg.K[Index::FIRING_CYCLE_US] = kFiringCycleUs;
  cinfo_msg.distortion_model = "VLP16";
  cinfo_msg.D.reserve(image.cols);

  // Unfortunately the buffer element is organized in columns, probably not very
  // cache-friendly
  for (int c = 0; c < image.cols; ++c) {
    const auto& tfseq = tfseqs[c];
    // D stores each azimuth angle
    cinfo_msg.D.push_back(tfseq.azimuth);

    // Fill in image
    for (int r = 0; r < image.rows; ++r) {
      // row 0 corresponds to max elevation (highest), row 15 corresponds to
      // min elevation (lowest) hence we flip row number
      // also data points are stored in laser ids which are interleaved, so we
      // need to convert to index first. See p54 table
      const auto rr = kFiringsPerFiringSequence - 1 - LaserId2Index(r);
      image.at<cv::Vec3b>(rr, c) =
          *(reinterpret_cast<const cv::Vec3b*>(&(tfseq.sequence.points[r])));
    }
  }

  cv_bridge::CvImage cv_image(header, "bgr8", image);
  return cv_image.toImageMsg();
}

void Decoder::PublishCloud(const sensor_msgs::ImageConstPtr& image_msg,
                           const sensor_msgs::CameraInfoConstPtr& cinfo_msg) {
  const auto cloud =
      ToCloud(image_msg, cinfo_msg, organized_, min_range_, max_range_);

  ROS_DEBUG("number of points in cloud: %zu", cloud->size());
  cloud_pub_.publish(cloud);
}

CloudT::Ptr ToCloud(const sensor_msgs::ImageConstPtr& image_msg,
                    const sensor_msgs::CameraInfoConstPtr& cinfo_msg,
                    bool organized, float min_range, float max_range) {
  CloudT::Ptr cloud(new CloudT);
  const auto image = cv_bridge::toCvShare(image_msg)->image;
  const auto& azimuths = cinfo_msg->D;

  cloud->header = pcl_conversions::toPCL(image_msg->header);
  cloud->reserve(image.total());

  const auto min_elevation = cinfo_msg->K[Index::MIN_ELEVATION];
  const auto max_elevation = cinfo_msg->K[Index::MAX_ELEVATION];
  const auto delta_elevation =
      (max_elevation - min_elevation) / (image.rows - 1);
  const auto distance_resolution = cinfo_msg->K[Index::DISTANCE_RESOLUTION];

  for (int r = 0; r < image.rows; ++r) {
    const auto* row_ptr = image.ptr<cv::Vec3b>(r);
    // Because index 0 is actually row 15
    const auto omega = max_elevation - r * delta_elevation;
    const auto cos_omega = std::cos(omega);
    const auto sin_omega = std::sin(omega);

    for (int c = 0; c < image.cols; ++c) {
      const cv::Vec3b& data = row_ptr[c];
      const auto alpha = azimuths[c];

      TwoBytes b2;
      b2.u8[0] = data[0];
      b2.u8[1] = data[1];
      const auto d = static_cast<float>(b2.u16) * distance_resolution;

      PointT p;
      if (d <= min_range || d >= max_range) {
        if (organized) {
          p.x = p.y = p.z = kPclNaN;
          cloud->points.push_back(p);
        }
      } else {
        // p.53 Figure 9-1 VLP-16 Sensor Coordinate System
        // Make x point forward and y point left, thus 0 azimuth is at x = 0 and
        // goes clockwise
        const auto x = d * cos_omega * std::sin(alpha);
        const auto y = d * cos_omega * std::cos(alpha);
        const auto z = d * sin_omega;

        p.x = y;
        p.y = -x;
        p.z = z;
        p.intensity = data[2];

        cloud->points.push_back(p);
      }
    }
  }

  if (organized) {
    cloud->width = image.cols;
    cloud->height = image.rows;
  } else {
    cloud->width = cloud->size();
    cloud->height = 1;
  }
  return cloud;
}

}  // namespace velodyne_puck
