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

#include <math.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace velodyne_puck {

using namespace sensor_msgs;
using namespace velodyne_msgs;

union TwoBytes {
  uint16_t u16;
  uint8_t u8[2];
};

Decoder::Decoder(const ros::NodeHandle& pnh)
    : pnh_(pnh), it_(pnh), cfg_server_(pnh) {
  //  pnh_.param("min_range", min_range_, 0.5);
  //  pnh_.param("max_range", max_range_, 100.0);
  //  ROS_ASSERT_MSG(min_range_ <= max_range_, "min_range > max_range");
  //  ROS_INFO("min_range: %f, max_range: %f", min_range_, max_range_);

  //  pnh_.param("organized", organized_, true);
  //  ROS_INFO("publish organized cloud: %s", organized_ ? "true" : "false");

  pnh_.param<std::string>("frame_id", frame_id_, "velodyne");
  ROS_INFO("Velodyne frame_id: %s", frame_id_.c_str());

  packet_sub_ =
      pnh_.subscribe<VelodynePacket>("packet", 100, &Decoder::PacketCb, this);
  cloud_pub_ = pnh_.advertise<PointCloud2>("cloud", 10);

  intensity_pub_ = it_.advertise("intensity", 1);
  range_pub_ = it_.advertise("range", 1);
  camera_pub_ = it_.advertiseCamera("image", 5);
  cfg_server_.setCallback(boost::bind(&Decoder::ConfigCb, this, _1, _2));
}

Decoder::Decoded Decoder::DecodePacket(const Packet* packet,
                                       double time) const {
  // Azimuth is clockwise, which is absurd
  // ^ y
  // | a /
  // |--/
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
      tfseq.azimuth = Raw2Azimuth(block.azimuth);  // need to fix half later
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

  if (!config_.full_sweep) {
    for (const auto& tfseq : decoded) {
      buffer_.push_back(tfseq);
      if (buffer_.size() >= static_cast<size_t>(config_.image_width)) {
        ROS_DEBUG("Publish fixed width with buffer size: %zu", buffer_.size());
        PublishBufferAndClear();
      }
    }
  } else {
    // Full scan mode 0~360
    // Check for a change of azimuth across 0
    float prev_azimuth = buffer_.empty() ? -1 : buffer_.back().azimuth;

    for (const auto& tfseq : decoded) {
      if (tfseq.azimuth < prev_azimuth) {
        // this indicates we cross the 0 azimuth angle
        // we are ready to publish what's in the buffer
        ROS_DEBUG("curr_azimuth: %f < %f prev_azimuth", rad2deg(tfseq.azimuth),
                  rad2deg(prev_azimuth));
        ROS_DEBUG("Publish full scan with buffer size: %zu", buffer_.size());
        PublishBufferAndClear();
      }

      buffer_.push_back(tfseq);
      prev_azimuth = tfseq.azimuth;
    }
  }
}

void Decoder::ConfigCb(VelodynePuckConfig& config, int level) {
  if (config.min_range > config.max_range) {
    ROS_WARN("min_range: %f > max_range: %f", config.min_range,
             config.max_range);
    config.min_range = config.max_range;
  }

  ROS_INFO(
      "Reconfigure Request: min_range: %f, max_range: %f, image_width: %d, "
      "organized: %s, full_sweep: %s",
      config.min_range, config.max_range, config.image_width,
      config.organized ? "True" : "False",
      config.full_sweep ? "True" : "False");

  config_ = config;
  buffer_.clear();
}

void Decoder::PublishBufferAndClear() {
  if (buffer_.empty()) return;

  const auto t1 = ros::Time::now();
  // Always convert to image data
  const CameraInfoPtr cinfo_msg(new CameraInfo);
  const auto image_msg = ToImageData(buffer_, *cinfo_msg);

  if (camera_pub_.getNumSubscribers()) {
    camera_pub_.publish(image_msg, cinfo_msg);
  }

  if (range_pub_.getNumSubscribers()) {
    PublishRange(image_msg);
  }

  if (intensity_pub_.getNumSubscribers()) {
    PublishIntensity(image_msg);
  }

  if (cloud_pub_.getNumSubscribers()) {
    PublishCloud(image_msg, cinfo_msg);
  }

  buffer_.clear();

  const auto time = (ros::Time::now() - t1).toSec();
  ROS_DEBUG("Total time for publish: %f", time);
}

void Decoder::PublishRange(const ImageConstPtr& image_msg) {
  const auto image = cv_bridge::toCvShare(image_msg)->image;
  cv::Mat range = cv::Mat::zeros(image.rows, image.cols, CV_16UC1);

  for (int r = 0; r < image.rows; ++r) {
    const auto* const row_ptr_image = image.ptr<cv::Vec3b>(r);
    auto* const row_ptr_range = range.ptr<uint16_t>(r);

    for (int c = 0; c < image.cols; ++c) {
      TwoBytes b2;
      b2.u8[0] = row_ptr_image[c][0];
      b2.u8[1] = row_ptr_image[c][1];
      row_ptr_range[c] = b2.u16;
    }
  }

  range_pub_.publish(
      cv_bridge::CvImage(image_msg->header, image_encodings::MONO16, range)
          .toImageMsg());
}

void Decoder::PublishIntensity(const ImageConstPtr& image_msg) {
  const auto image = cv_bridge::toCvShare(image_msg)->image;
  cv::Mat intensity = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);

  for (int r = 0; r < image.rows; ++r) {
    const auto* const row_ptr_image = image.ptr<cv::Vec3b>(r);
    auto* const row_ptr_intensity = intensity.ptr<uint8_t>(r);

    for (int c = 0; c < image.cols; ++c) {
      row_ptr_intensity[c] = row_ptr_image[c][2];
    }
  }

  intensity_pub_.publish(
      cv_bridge::CvImage(image_msg->header, image_encodings::MONO8, intensity)
          .toImageMsg());
}

ImagePtr Decoder::ToImageData(const std::vector<FiringSequenceStamped>& fseqs,
                              CameraInfo& cinfo_msg) const {
  std_msgs::Header header;
  header.stamp = ros::Time(fseqs[0].time);
  header.frame_id = frame_id_;

  cv::Mat image =
      cv::Mat::zeros(kFiringsPerFiringSequence, fseqs.size(), CV_8UC3);
  ROS_DEBUG("image size: %d x %d", image.rows, image.cols);

  //  sensor_msgs::CameraInfoPtr cinfo_msg(new sensor_msgs::CameraInfo);
  cinfo_msg.header = header;
  cinfo_msg.height = image.rows;
  cinfo_msg.width = image.cols;
  cinfo_msg.K[0] = kMinElevation;
  cinfo_msg.K[1] = kMaxElevation;
  cinfo_msg.R[0] = kDistanceResolution;
  cinfo_msg.P[0] = kFiringCycleUs;
  cinfo_msg.distortion_model = "VLP16";
  cinfo_msg.D.reserve(image.cols);

  // Unfortunately the buffer element is organized in columns, probably not very
  // cache-friendly
  for (int c = 0; c < image.cols; ++c) {
    const auto& tfseq = fseqs[c];
    // D stores each azimuth angle
    cinfo_msg.D.push_back(tfseq.azimuth);

    // Fill in image
    for (int r = 0; r < image.rows; ++r) {
      // row 0 corresponds to max elevation (highest), row 15 corresponds to
      // min elevation (lowest) hence we flip row number
      // also data points are stored in laser ids which are interleaved, so we
      // need to convert to index first. See p54 table
      //      const auto rr = kFiringsPerFiringSequence - 1 - LaserId2Index(r);
      //      image.at<cv::Vec3b>(rr, c) =
      //          *(reinterpret_cast<const
      //          cv::Vec3b*>(&(tfseq.sequence.points[r])));
      const auto rr = Index2LaserId(kFiringsPerFiringSequence - 1 - r);
      image.at<cv::Vec3b>(r, c) =
          *(reinterpret_cast<const cv::Vec3b*>(&(tfseq.sequence.points[rr])));
    }
  }

  return cv_bridge::CvImage(header, image_encodings::BGR8, image).toImageMsg();
}

void Decoder::PublishCloud(const ImageConstPtr& image_msg,
                           const CameraInfoConstPtr& cinfo_msg) {
  const auto cloud = ToCloud(image_msg, cinfo_msg, config_.organized,
                             config_.min_range, config_.max_range);

  ROS_DEBUG("number of points in cloud: %zu", cloud->size());
  cloud_pub_.publish(cloud);
}

CloudT::Ptr ToCloud(const ImageConstPtr& image_msg,
                    const CameraInfoConstPtr& cinfo_msg, bool organized,
                    float min_range, float max_range) {
  CloudT::Ptr cloud(new CloudT);
  const auto image = cv_bridge::toCvShare(image_msg)->image;
  const auto& azimuths = cinfo_msg->D;

  cloud->header = pcl_conversions::toPCL(image_msg->header);
  cloud->reserve(image.total());

  const float min_elevation = cinfo_msg->K[0];
  const float max_elevation = cinfo_msg->K[1];
  const float delta_elevation =
      (max_elevation - min_elevation) / (image.rows - 1);
  const float distance_resolution = cinfo_msg->R[0];

  // Precompute sin cos
  std::vector<std::pair<float, float>> sin_cos;
  sin_cos.reserve(azimuths.size());
  for (size_t i = 0; i < azimuths.size(); ++i) {
    //    sincos(azimuths[i], &(sin_cos[i].first), &(sin_cos[i].second));
    sin_cos.emplace_back(std::sin(azimuths[i]), std::cos(azimuths[i]));
  }

  for (int r = 0; r < image.rows; ++r) {
    const auto* const row_ptr = image.ptr<cv::Vec3b>(r);
    // Because image row 0 is the highest laser point
    const float omega = max_elevation - r * delta_elevation;
    const auto cos_omega = std::cos(omega);
    const auto sin_omega = std::sin(omega);

    for (int c = 0; c < image.cols; ++c) {
      const cv::Vec3b& data = row_ptr[c];
      // const auto alpha = azimuths[c];

      TwoBytes b2;
      b2.u8[0] = data[0];
      b2.u8[1] = data[1];
      const float d = static_cast<float>(b2.u16) * distance_resolution;

      PointT p;
      if (d < min_range || d > max_range) {
        if (organized) {
          p.x = p.y = p.z = kPclNaN;
          cloud->points.push_back(p);
        }
      } else {
        // p.53 Figure 9-1 VLP-16 Sensor Coordinate System
        // const auto x = d * cos_omega * std::sin(alpha);
        // const auto y = d * cos_omega * std::cos(alpha);
        const auto x = d * cos_omega * sin_cos[c].first;
        const auto y = d * cos_omega * sin_cos[c].second;
        const auto z = d * sin_omega;

        // original velodyne frame is x right y forward
        // we make x forward and y left, thus 0 azimuth is at x = 0 and
        // goes clockwise
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
