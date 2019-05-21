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
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>

namespace velodyne_puck_decoder {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using velodyne_puck_msgs::VelodynePacket;
using velodyne_puck_msgs::VelodynePoint;
using velodyne_puck_msgs::VelodyneScan;

/// Compute total points in a sweep
size_t TotalPoints(const VelodyneSweep& sweep) {
  size_t n = 0;
  for (const VelodyneScan& scan : sweep.scans) {
    n += scan.points.size();
  }
  return n;
}

VelodynePuckDecoder::VelodynePuckDecoder(const ros::NodeHandle& n,
                                         const ros::NodeHandle& pn)
    : nh(n), pnh(pn), it(pn), sweep_data(new VelodyneSweep()) {
  pnh.param("min_range", min_range, 0.5);
  pnh.param("max_range", max_range, 100.0);
  ROS_INFO("min_range: %f, max_range: %f", min_range, max_range);

  //  int rpm;
  //  pnh.param("rpm", rpm, 300);
  //  ROS_INFO("azimuth resolution %f degree for rpm %d",
  //           AzimuthResolutionDegree(rpm), rpm);

  pnh.param<std::string>("frame_id", frame_id, "velodyne");
  ROS_INFO("Velodyne frame_id: %s", frame_id.c_str());

  packet_sub = pnh.subscribe<VelodynePacket>(
      "packet", 100, &VelodynePuckDecoder::PacketCb, this);

  sweep_pub = pnh.advertise<VelodyneSweep>("sweep", 10);
  cloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("cloud", 10);
  cloud2_pub = pnh.advertise<sensor_msgs::PointCloud2>("cloud_new", 10);
  camera_pub = it.advertiseCamera("image", 10);
}

bool VelodynePuckDecoder::Initialize() {
  // Fill in the altitude for each scan.
  for (size_t laser_id = 0; laser_id < kFiringsPerFiringSequence; ++laser_id) {
    const auto scan_index = LaserId2Index(laser_id);
    const auto elevation = kMinElevation + scan_index * kDeltaElevation;
    sweep_data->scans[scan_index].elevation = elevation;
  }

  return true;
}

bool VelodynePuckDecoder::CheckData(const RawPacket* packet) {
  for (int i = 0; i < kDataBlocksPerPacket; ++i) {
    if (packet->blocks[i].flag != UPPER_BANK) {
      ROS_WARN("Skip invalid VLP-16 packet: block %d header is %x", i,
               packet->blocks[i].flag);
      return false;
    }
  }
  return true;
}

void VelodynePuckDecoder::DecodePacket(const RawPacket* packet) {
  // Compute the azimuth angle for each firing.
  for (size_t fir_idx = 0; fir_idx < kFiringSequencesPerPacket /*24*/;
       fir_idx += 2) {
    size_t blk_idx = fir_idx / 2;
    const auto raw_azimuth = packet->blocks[blk_idx].azimuth;
    firings[fir_idx].firing_azimuth = Azimuth(raw_azimuth);
    //    ROS_INFO_STREAM("Raw azimuth " << raw_azimuth);
    ROS_WARN_STREAM_COND(raw_azimuth > kMaxRawAzimuth,
                         "raw aimuth too big " << raw_azimuth);
  }

  // Interpolate the azimuth values
  for (size_t fir_idx = 1; fir_idx < kFiringSequencesPerPacket; fir_idx += 2) {
    size_t lfir_idx = fir_idx - 1;
    size_t rfir_idx = fir_idx + 1;

    if (fir_idx == kFiringSequencesPerPacket - 1) {
      lfir_idx = fir_idx - 3;
      rfir_idx = fir_idx - 1;
    }

    double azimuth_diff =
        firings[rfir_idx].firing_azimuth - firings[lfir_idx].firing_azimuth;
    azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 2 * M_PI : azimuth_diff;

    firings[fir_idx].firing_azimuth =
        firings[fir_idx - 1].firing_azimuth + azimuth_diff / 2.0;
    firings[fir_idx].firing_azimuth =
        firings[fir_idx].firing_azimuth > 2 * M_PI
            ? firings[fir_idx].firing_azimuth - 2 * M_PI
            : firings[fir_idx].firing_azimuth;
    const auto az = firings[fir_idx].firing_azimuth;
    ROS_WARN_STREAM_COND(az < 0 || az > (2 * M_PI), "1 azimuth wrong " << az);
  }

  // Fill in the distance and intensity for each firing.
  for (size_t blk_idx = 0; blk_idx < kDataBlocksPerPacket; ++blk_idx) {
    const RawBlock& raw_block = packet->blocks[blk_idx];

    for (size_t blk_fir_idx = 0; blk_fir_idx < kFiringSequencesPerDataBlock;
         ++blk_fir_idx) {
      size_t fir_idx = blk_idx * kFiringSequencesPerDataBlock + blk_fir_idx;
      auto& firing = firings[fir_idx];

      double azimuth_diff = 0.0;
      if (fir_idx < kFiringSequencesPerPacket - 1)
        azimuth_diff =
            firings[fir_idx + 1].firing_azimuth - firing.firing_azimuth;
      else
        azimuth_diff =
            firing.firing_azimuth - firings[fir_idx - 1].firing_azimuth;

      for (size_t scan_fir_idx = 0; scan_fir_idx < kFiringsPerFiringSequence;
           ++scan_fir_idx) {
        size_t byte_idx =
            kPointBytes *
            (kFiringsPerFiringSequence * blk_fir_idx + scan_fir_idx);

        // Azimuth
        firing.azimuth[scan_fir_idx] =
            firing.firing_azimuth +
            (scan_fir_idx * kSingleFiringUs / kFiringCycleUs) * azimuth_diff;

        const auto az = firing.firing_azimuth;
        ROS_WARN_STREAM_COND(az < 0 || az > (2 * M_PI),
                             "2 azimuth wrong " << az);

        // Distance
        TwoBytes raw_distance;
        raw_distance.u8[0] = raw_block.data[byte_idx];
        raw_distance.u8[1] = raw_block.data[byte_idx + 1];
        firings[fir_idx].distance[scan_fir_idx] =
            static_cast<double>(raw_distance.u16) * kDistanceResolution;

        // Intensity
        firings[fir_idx].intensity[scan_fir_idx] =
            static_cast<double>(raw_block.data[byte_idx + 2]);
      }
    }
  }
}

VelodynePuckDecoder::Decoded VelodynePuckDecoder::DecodePacket(
    const Packet* packet, double time) {
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
        "instead got %u",
        return_mode);
    ros::shutdown();
  }
  const auto product_id = packet->factory[1];
  if (product_id != 34) {
    ROS_ERROR("product id must be VLP-16 or Puck Lite (34), instead got %u",
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

    // Fill in decoded
    // for each firing sequence in the data block, 2
    for (int fsi = 0; fsi < kFiringSequencesPerDataBlock; ++fsi) {
      // Index into decoded, hardcode 2 for now
      const auto di = bi * 2 + fsi;
      auto& tfseq = decoded[di];
      // Assume all firings within each firing sequence occur at the same time
      tfseq.time = time + di * kFiringCycleUs * 1e-6;
      tfseq.azimuth = Azimuth(block.azimuth);  // Half of the azimuth is wrong
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
    ROS_WARN_COND(std::abs(azimuth_diff_deg - 0.1) > 1e-2,
                  "azimuth_diff too big: %f deg", azimuth_diff_deg);
    azimuth += azimuth_diff;
    if (azimuth > kTau) {
      azimuth -= kTau;
    }

    decoded[di].azimuth = azimuth;
  }

  return decoded;
}

void VelodynePuckDecoder::PacketCb(const VelodynePacketConstPtr& packet_msg) {
  // Convert the msg to the raw packet type.
  //  const RawPacket* packet = (const RawPacket*)(&(packet_msg->data[0]));
  const auto* packet =
      reinterpret_cast<const RawPacket*>(&(packet_msg->data[0]));

  // Check if the packet is valid
  if (!CheckData(packet)) return;

  // Decode the packet
  DecodePacket(packet);

  // Find the start of a new revolution
  //    If there is one, new_sweep_start will be the index of the start firing,
  //    otherwise, new_sweep_start will be FIRINGS_PER_PACKET.
  size_t new_sweep_start = 0;
  do {
    if (firings[new_sweep_start].firing_azimuth < last_azimuth)
      break;
    else {
      last_azimuth = firings[new_sweep_start].firing_azimuth;
      ++new_sweep_start;
    }
  } while (new_sweep_start < kFiringSequencesPerPacket);

  // The first sweep may not be complete. So, the firings with
  // the first sweep will be discarded. We will wait for the
  // second sweep in order to find the 0 azimuth angle.
  size_t start_fir_idx = 0;
  size_t end_fir_idx = new_sweep_start;
  if (is_first_sweep && new_sweep_start == kFiringSequencesPerPacket) {
    // The first sweep has not ended yet.
    return;
  } else {
    if (is_first_sweep) {
      is_first_sweep = false;
      start_fir_idx = new_sweep_start;
      end_fir_idx = kFiringSequencesPerPacket;
      sweep_start_time = packet_msg->stamp.toSec() +
                         kFiringCycleUs * (end_fir_idx - start_fir_idx) * 1e-6;
    }
  }

  for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
    const auto& firing = firings[fir_idx];
    for (size_t laser_id = 0; laser_id < kFiringsPerFiringSequence;
         ++laser_id) {
      // Compute the time of the point
      double time = packet_start_time + kFiringCycleUs * fir_idx +
                    kSingleFiringUs * laser_id;

      // Remap the index of the scan
      const int scan_index = LaserId2Index(laser_id);
      //          laser_id % 2 == 0 ? laser_id / 2 : laser_id / 2 + 8;
      // Pack the data into point msg
      VelodynePoint new_point;
      new_point.time = time;
      new_point.azimuth = firing.azimuth[laser_id];
      new_point.distance = firing.distance[laser_id];
      new_point.intensity = firing.intensity[laser_id];
      sweep_data->scans[scan_index].points.push_back(new_point);
    }
  }

  packet_start_time += kFiringCycleUs * (end_fir_idx - start_fir_idx);

  // A new sweep begins
  if (end_fir_idx != kFiringSequencesPerPacket) {
    // Publish the last revolution
    sweep_data->header.stamp = ros::Time(sweep_start_time);
    sweep_pub.publish(sweep_data);

    if (cloud_pub.getNumSubscribers() > 0) {
      PublishCloud(*sweep_data);
    }

    sweep_data.reset(new VelodyneSweep());
    for (size_t laser_id = 0; laser_id < kFiringsPerFiringSequence;
         ++laser_id) {
      const auto scan_index = LaserId2Index(laser_id);
      const auto elevation = kMinElevation + scan_index * kDeltaElevation;
      sweep_data->scans[scan_index].elevation = elevation;
    }

    // Prepare the next revolution
    sweep_start_time = packet_msg->stamp.toSec() +
                       kFiringCycleUs * (end_fir_idx - start_fir_idx) * 1e-6;
    packet_start_time = 0.0;
    last_azimuth = firings[kFiringSequencesPerPacket - 1].firing_azimuth;

    start_fir_idx = end_fir_idx;
    end_fir_idx = kFiringSequencesPerPacket;

    for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
      const auto& firing = firings[fir_idx];
      for (size_t laser_id = 0; laser_id < kFiringsPerFiringSequence;
           ++laser_id) {
        // Compute the time of the point
        double time = packet_start_time +
                      kFiringCycleUs * (fir_idx - start_fir_idx) +
                      kSingleFiringUs * laser_id;

        // Remap the index of the scan
        int scan_index = LaserId2Index(laser_id);

        // Pack the data into point msg
        VelodynePoint new_point;
        new_point.time = time;
        new_point.azimuth = firing.azimuth[laser_id];
        new_point.distance = firing.distance[laser_id];
        new_point.intensity = firing.intensity[laser_id];
        sweep_data->scans[scan_index].points.push_back(new_point);
      }
    }

    packet_start_time += kFiringCycleUs * (end_fir_idx - start_fir_idx);
  }

  // My stuff
  {
    const auto* my_packet =
        reinterpret_cast<const Packet*>(&(packet_msg->data[0]));
    const auto decoded = DecodePacket(my_packet, packet_msg->stamp.toSec());

    // Check for a change of azimuth across 0
    float prev_azimuth = buffer_.empty() ? -1 : buffer_.back().azimuth;
    for (const auto& tfseq : decoded) {
      if (tfseq.azimuth < prev_azimuth) {
        // this indicates we cross the 0 azimuth angle
        // we are ready to publish this
        ROS_INFO("curr_azimuth: %f < %f prev_azimuth", rad2deg(tfseq.azimuth),
                 rad2deg(prev_azimuth));
        ROS_INFO("buffer size: %zu", buffer_.size());

        if (buffer_.empty()) continue;

        const auto range_image = ToRangeImage(buffer_);
        PublishImage(range_image);
        ROS_INFO("after pub image range_image: %d %d, %zu", range_image.first->height,
                 range_image.first->width, range_image.first->data.size());
        PublishCloud(range_image);

        buffer_.clear();
        prev_azimuth = -1;
      } else {
        // azimuth keep increasing so keep adding to buffer
        buffer_.push_back(tfseq);
        prev_azimuth = buffer_.back().azimuth;
      }
    }
  }
}

PointT SphericalToEuclidean(const VelodynePoint& vp, float omega) {
  PointT p;

  const auto cos_omega = std::cos(omega);
  const auto d = vp.distance;
  const auto alpha = vp.azimuth;

  // p.53 Figure 9-1 VLP-16 Sensor Coordinate System
  const auto x = d * cos_omega * std::sin(alpha);
  const auto y = d * cos_omega * std::cos(alpha);
  const auto z = d * std::sin(omega);

  p.x = y;
  p.y = -x;
  p.z = z;
  p.intensity = vp.intensity;

  return p;
}

void VelodynePuckDecoder::PublishCloud(const VelodyneSweep& sweep_msg) {
  CloudT::Ptr cloud = boost::make_shared<CloudT>();

  cloud->header = pcl_conversions::toPCL(sweep_msg.header);
  cloud->header.frame_id = frame_id;
  cloud->height = 1;
  cloud->reserve(TotalPoints(sweep_msg));

  for (const VelodyneScan& scan : sweep_msg.scans) {
    // The first and last point in each scan is ignored, which
    // seems to be corrupted based on the received data.
    // TODO: The two end points should be removed directly in the scans.
    if (scan.points.size() <= 2) continue;
    for (size_t j = 1; j < scan.points.size() - 1; ++j) {
      // TODO: compute here instead of saving them in scan, waste space
      if (!IsPointInRange(scan.points[j].distance)) continue;
      const auto point = SphericalToEuclidean(scan.points[j], scan.elevation);
      // cloud->push_back does extra work, so we don't use it
      cloud->points.push_back(point);
    }
  }

  cloud->width = cloud->size();
  cloud_pub.publish(cloud);
  ROS_DEBUG("Total cloud %zu", cloud->size());
}

VelodynePuckDecoder::RangeImage VelodynePuckDecoder::ToRangeImage(
    const std::vector<TimedFiringSequence>& tfseqs) const {
  std_msgs::Header header;
  header.stamp = ros::Time(tfseqs[0].time);
  header.frame_id = frame_id;

  cv::Mat image =
      cv::Mat::zeros(kFiringsPerFiringSequence, tfseqs.size(), CV_8UC3);
  ROS_INFO("image size: %d x %d", image.rows, image.cols);

  sensor_msgs::CameraInfoPtr cinfo_msg(new sensor_msgs::CameraInfo);
  cinfo_msg->header = header;
  cinfo_msg->height = image.rows;
  cinfo_msg->width = image.cols;
  cinfo_msg->K[0] = kMinElevation;
  cinfo_msg->K[1] = kMaxElevation;
  cinfo_msg->K[2] = kDistanceResolution;
  cinfo_msg->distortion_model = "VLP16";
  cinfo_msg->D.reserve(image.cols);

  // Unfortunately the buffer element is organized in columns, probably not very
  // cache-friendly

  for (int c = 0; c < image.cols; ++c) {
    const auto& tfseq = tfseqs[c];
    // D stores each azimuth angle
    cinfo_msg->D.push_back(tfseq.azimuth);

    // Fill in image
    for (int r = 0; r < image.rows; ++r) {
      // NOTE:
      // row 0 corresponds to max elevation, row 15 corresponds to min elevation
      // hence we flip row number
      // also data points are stored in laser ids which are interleaved
      // See p54 table
      const auto rr = LaserId2Index(r, true);
      image.at<cv::Vec3b>(rr, c) =
          *(reinterpret_cast<const cv::Vec3b*>(&(tfseq.sequence.points[r])));
      // image.at<uint16_t>(rr, c) = tfseq.sequence.points[r].distance;
      // image.at<uint16_t>(rr, c) = tfseq.sequence.points[r].reflectivity;
    }
  }

  cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8,
                              image);

  return std::make_pair(cv_image.toImageMsg(), cinfo_msg);
}

void VelodynePuckDecoder::PublishImage(const RangeImage& range_image) {
  camera_pub.publish(range_image.first, range_image.second);
  ROS_INFO("pub image range_image: %d %d, %zu", range_image.first->height,
           range_image.first->width, range_image.first->data.size());
}

void VelodynePuckDecoder::PublishCloud(const RangeImage& range_image) {
  // Here we convert range_image to point cloud and profit!!
  bool organized = true;

  CloudT::Ptr cloud = boost::make_shared<CloudT>();

  ROS_INFO("range_image: %d %d, %zu", range_image.first->height,
           range_image.first->width, range_image.first->data.size());
  const auto image = cv_bridge::toCvShare(range_image.first)->image;
  const auto& azimuths = range_image.second->D;
  ROS_INFO("image size %d x %d", image.rows, image.cols);

  cloud->header = pcl_conversions::toPCL(range_image.first->header);
  cloud->reserve(image.total());

  for (int r = 0; r < image.rows; ++r) {
    const auto* row_ptr = image.ptr<cv::Vec3b>(r);
    // Because index 0 is actually row 15
    const auto omega = kMaxElevation - r * kDeltaElevation;
    const auto cos_omega = std::cos(omega);
    const auto sin_omega = std::sin(omega);

    for (int c = 0; c < image.cols; ++c) {
      const cv::Vec3b& data = row_ptr[c];
      const auto alpha = azimuths[c];

      TwoBytes b2;
      b2.u8[0] = data[0];
      b2.u8[1] = data[1];
      const auto d = static_cast<float>(b2.u16) * kDistanceResolution;

      const auto x = d * cos_omega * std::sin(alpha);
      const auto y = d * cos_omega * std::cos(alpha);
      const auto z = d * sin_omega;

      PointT p;
      p.x = y;
      p.y = -x;
      p.z = z;
      p.intensity = data[2];
      cloud->points.push_back(p);
    }
  }

  cloud->width = image.cols;
  cloud->height = image.rows;

  ROS_INFO("number of points in cloud: %zu", cloud->size());
  cloud2_pub.publish(cloud);
}

}  //  namespace velodyne_puck_decoder
