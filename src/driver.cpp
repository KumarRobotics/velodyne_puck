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

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cmath>

#include "constants.h"
#include "driver.h"

namespace velodyne_puck {

/// Constants
static constexpr uint16_t kUdpPort = 2368;
static constexpr size_t kPacketSize = sizeof(VelodynePacket().data);
static constexpr int kError = -1;

// p49 8.2.1
static constexpr double kDelayPerPacketUs =
    kFiringSequencesPerPacket * kFiringCycleUs;
static constexpr double kPacketsPerSecond = 1e6 / kDelayPerPacketUs;

/// VelodynePuckDriver
Driver::Driver(const ros::NodeHandle &n, const ros::NodeHandle &pn)
    : nh(n), pnh(pn) {
  ROS_INFO("packet size: %zu", kPacketSize);
  pnh.param("device_ip", device_ip_str, std::string("192.168.1.201"));
  ROS_INFO("device_ip: %s", device_ip_str.c_str());

  if (inet_aton(device_ip_str.c_str(), &device_ip) == 0) {
    // inet_aton() returns nonzero if the address is valid, zero if not.
    ROS_FATAL("Invalid device ip: %s", device_ip_str.c_str());
    ros::shutdown();
  }

  // ROS diagnostics
  diagnostics.setHardwareID("Velodyne_VLP16");
  // VLP16 publishs 0.3 million points per second.
  // Each packet contains 12 blocks. And each block
  // contains 32 points. Together provides the
  // packet rate.
  //  const double diag_freq = 300000.0 / (12 * 32);

  // 8.2.1 Data Packet Rate
  // There are 24 firing cycles in a data packet.
  // 24 x 55.296 μs = 1.327 ms is the accumulation delay per packet.
  // 1 packet/1.327 ms = 753.5 packets/second
  const double diag_freq = kPacketsPerSecond;
  diag_max_freq = diag_min_freq = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic.reset(new TopicDiagnostic(
      "packet", diagnostics,
      FrequencyStatusParam(&diag_min_freq, &diag_max_freq, 0.1, 10),
      TimeStampStatusParam()));

  // Output
  packet_pub = pnh.advertise<VelodynePacket>("packet", 10);

  if (!OpenUdpPort()) {
    ROS_ERROR("Failed to open UDP Port");
  }

  ROS_INFO("Successfully opened UDP Port at %s", device_ip_str.c_str());
}

Driver::~Driver() {
  if (close(socket_id)) {
    ROS_INFO("Close socket %d at %s", socket_id, device_ip_str.c_str());
  } else {
    ROS_ERROR("Failed to close socket %d at %s", socket_id,
              device_ip_str.c_str());
  }
}

bool Driver::OpenUdpPort() {
  socket_id = socket(PF_INET, SOCK_DGRAM, 0);
  if (socket_id == -1) {
    perror("socket");
    ROS_ERROR("Failed to create socket");
    return false;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(kUdpPort);    // short, in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(socket_id, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
    perror("bind");  // TODO: ROS_ERROR errno
    ROS_ERROR("Failed to bind to socket %d", socket_id);
    return false;
  }

  if (fcntl(socket_id, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    perror("non-block");
    ROS_ERROR("Failed to set socket to non-blocking");
    return false;
  }

  return true;
}

int Driver::ReadPacket(VelodynePacket &packet) const {
  const auto time_before = ros::Time::now();

  struct pollfd fds[1];
  fds[0].fd = socket_id;
  fds[0].events = POLLIN;
  const int timeout_ms = 1000;  // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  while (true) {
    // Unfortunately, the Linux kernel recvfrom() implementation
    // uses a non-interruptible sleep() when waiting for data,
    // which would cause this method to hang if the device is not
    // providing data.  We poll() the device first to make sure
    // the recvfrom() will not block.
    //
    // Note, however, that there is a known Linux kernel bug:
    //
    //   Under Linux, select() may report a socket file descriptor
    //   as "ready for reading", while nevertheless a subsequent
    //   read blocks.  This could for example happen when data has
    //   arrived but upon examination has wrong checksum and is
    //   discarded.  There may be other circumstances in which a
    //   file descriptor is spuriously reported as ready.  Thus it
    //   may be safer to use O_NONBLOCK on sockets that should not
    //   block.

    // poll() until input available
    do {
      const int retval = poll(fds, 1, timeout_ms);

      if (retval < 0) {
        // poll() error?
        if (errno != EINTR) ROS_ERROR("poll() error: %s", strerror(errno));
        return kError;
      } else if (retval == 0) {
        // poll() timeout?
        ROS_WARN("Velodyne poll() timeout");
        return kError;
      }

      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
          (fds[0].revents & POLLNVAL)) {
        // device error?
        ROS_ERROR("poll() reports Velodyne error");
        return kError;
      }
    } while ((fds[0].revents & POLLIN) == 0);

    // Receive packets that should now be available from the
    // socket using a blocking read.
    const ssize_t nbytes =
        recvfrom(socket_id, &packet.data[0], kPacketSize, 0,
                 (sockaddr *)&sender_address, &sender_address_len);

    if (nbytes < 0) {
      if (errno != EWOULDBLOCK) {
        perror("recvfail");
        ROS_ERROR("Failed to read from socket");
        return kError;
      }
    } else if ((size_t)nbytes == kPacketSize) {
      // read successful,
      // if packet is not from the lidar scanner we selected by IP,
      // continue otherwise we are done
      if (device_ip_str != "" &&
          sender_address.sin_addr.s_addr != device_ip.s_addr)
        continue;
      else
        break;  // done
    }

    ROS_DEBUG_STREAM("incomplete Velodyne packet read: " << nbytes << " bytes");
  }

  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred.
  const auto time_after = ros::Time::now();
  // Usually take around 0.0012s on my pc
  //  ROS_INFO("time: %f", time_after - time_before);
  //  packet.stamp = ros::Time((time_after + time_before) / 2.0);
  // The ros velodyne driver uses average time to as the time of the packet.
  // We just use the starting time, and the actual time is this time + some time
  // delay of communication
  packet.stamp = time_before;

  return 0;
}

bool Driver::Poll() {
  auto packet = boost::make_shared<VelodynePacket>();

  while (true) {
    // keep reading until full packet received
    const int rc = ReadPacket(*packet);
    if (rc == 0) break;        // got a full packet?
    if (rc < 0) return false;  // end of file reached?
  }

  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full Velodyne scan.");
  packet_pub.publish(packet);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic->tick(packet->stamp);
  diagnostics.update();

  return true;
}

}  // namespace velodyne_puck
