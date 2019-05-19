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

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

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
    : nh(n), pnh(pn), sweep_data(new VelodyneSweep()) {
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
}

bool VelodynePuckDecoder::Initialize() {
  // Fill in the altitude for each scan.
  for (size_t laser_id = 0; laser_id < kFiringsPerSequence; ++laser_id) {
    const auto scan_index = LaserId2Index(laser_id);
    const auto elevation = kMinElevation + scan_index * kDeltaElevation;
    sweep_data->scans[scan_index].elevation = elevation;
  }

  return true;
}

bool VelodynePuckDecoder::checkPacketValidity(const RawPacket* packet) {
  for (int i = 0; i < kBlocksPerPacket; ++i) {
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
  for (size_t fir_idx = 0; fir_idx < kFiringsPerPacket /*24*/; fir_idx += 2) {
    size_t blk_idx = fir_idx / 2;
    const auto raw_azimuth = packet->blocks[blk_idx].azimuth;
    firings[fir_idx].firing_azimuth = RawAzimuthToFloat(raw_azimuth);
    //    ROS_INFO_STREAM("Raw azimuth " << raw_azimuth);
    ROS_WARN_STREAM_COND(raw_azimuth > 35999,
                         "raw aimuth too big " << raw_azimuth);
  }

  // Interpolate the azimuth values
  for (size_t fir_idx = 1; fir_idx < kFiringsPerPacket; fir_idx += 2) {
    size_t lfir_idx = fir_idx - 1;
    size_t rfir_idx = fir_idx + 1;

    if (fir_idx == kFiringsPerPacket - 1) {
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
  for (size_t blk_idx = 0; blk_idx < kBlocksPerPacket; ++blk_idx) {
    const RawBlock& raw_block = packet->blocks[blk_idx];

    for (size_t blk_fir_idx = 0; blk_fir_idx < kSequencePerBlock;
         ++blk_fir_idx) {
      size_t fir_idx = blk_idx * kSequencePerBlock + blk_fir_idx;
      auto& firing = firings[fir_idx];

      double azimuth_diff = 0.0;
      if (fir_idx < kFiringsPerPacket - 1)
        azimuth_diff =
            firings[fir_idx + 1].firing_azimuth - firing.firing_azimuth;
      else
        azimuth_diff =
            firing.firing_azimuth - firings[fir_idx - 1].firing_azimuth;

      for (size_t scan_fir_idx = 0; scan_fir_idx < kFiringsPerSequence;
           ++scan_fir_idx) {
        size_t byte_idx =
            kPointBytes * (kFiringsPerSequence * blk_fir_idx + scan_fir_idx);

        // Azimuth
        firing.azimuth[scan_fir_idx] =
            firing.firing_azimuth +
            (scan_fir_idx * kSingleFiringUs / kFiringCycleUs) * azimuth_diff;

        const auto az = firing.firing_azimuth;
        ROS_WARN_STREAM_COND(az < 0 || az > (2 * M_PI),
                             "2 azimuth wrong " << az);

        // Distance
        TwoBytes raw_distance;
        raw_distance.bytes[0] = raw_block.data[byte_idx];
        raw_distance.bytes[1] = raw_block.data[byte_idx + 1];
        firings[fir_idx].distance[scan_fir_idx] =
            static_cast<double>(raw_distance.distance) * kDistanceResolution;

        // Intensity
        firings[fir_idx].intensity[scan_fir_idx] =
            static_cast<double>(raw_block.data[byte_idx + 2]);
      }
    }
  }
}

void VelodynePuckDecoder::PacketCb(const VelodynePacketConstPtr& packet_msg) {
  // Convert the msg to the raw packet type.
  const RawPacket* packet = (const RawPacket*)(&(packet_msg->data[0]));

  // Check if the packet is valid
  if (!checkPacketValidity(packet)) return;

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
  } while (new_sweep_start < kFiringsPerPacket);

  // The first sweep may not be complete. So, the firings with
  // the first sweep will be discarded. We will wait for the
  // second sweep in order to find the 0 azimuth angle.
  size_t start_fir_idx = 0;
  size_t end_fir_idx = new_sweep_start;
  if (is_first_sweep && new_sweep_start == kFiringsPerPacket) {
    // The first sweep has not ended yet.
    return;
  } else {
    if (is_first_sweep) {
      is_first_sweep = false;
      start_fir_idx = new_sweep_start;
      end_fir_idx = kFiringsPerPacket;
      sweep_start_time = packet_msg->stamp.toSec() +
                         kFiringCycleUs * (end_fir_idx - start_fir_idx) * 1e-6;
    }
  }

  for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
    const auto& firing = firings[fir_idx];
    for (size_t laser_id = 0; laser_id < kFiringsPerSequence; ++laser_id) {
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
  if (end_fir_idx != kFiringsPerPacket) {
    // Publish the last revolution
    sweep_data->header.stamp = ros::Time(sweep_start_time);
    sweep_pub.publish(sweep_data);

    if (cloud_pub.getNumSubscribers() > 0) {
      PublishCloud(*sweep_data);
    }

    sweep_data.reset(new VelodyneSweep());
    for (size_t laser_id = 0; laser_id < kFiringsPerSequence; ++laser_id) {
      const auto scan_index = LaserId2Index(laser_id);
      const auto elevation = kMinElevation + scan_index * kDeltaElevation;
      sweep_data->scans[scan_index].elevation = elevation;
    }

    // Prepare the next revolution
    sweep_start_time = packet_msg->stamp.toSec() +
                       kFiringCycleUs * (end_fir_idx - start_fir_idx) * 1e-6;
    packet_start_time = 0.0;
    last_azimuth = firings[kFiringsPerPacket - 1].firing_azimuth;

    start_fir_idx = end_fir_idx;
    end_fir_idx = kFiringsPerPacket;

    for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
      const auto& firing = firings[fir_idx];
      for (size_t laser_id = 0; laser_id < kFiringsPerSequence; ++laser_id) {
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
  ROS_INFO("Cloud");
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

}  //  namespace velodyne_puck_decoder
