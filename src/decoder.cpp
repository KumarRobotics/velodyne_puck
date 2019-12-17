#include "constants.h"

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_puck/VelodynePuckConfig.h>

namespace velodyne_puck {

using namespace sensor_msgs;
using namespace velodyne_msgs;

using Veckf = cv::Vec<float, 2>;
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

/// Used for indexing into packet and image, (NOISE not used now)
enum Index { RANGE = 0, INTENSITY = 1, AZIMUTH = 2, NOISE = 3 };

class Decoder {
 public:
  /// Number of channels for image data
  static constexpr int kChannels = 2;  // (range [m], intensity)

  explicit Decoder(const ros::NodeHandle& pnh);

  Decoder(const Decoder&) = delete;
  Decoder operator=(const Decoder&) = delete;

  void PacketCb(const VelodynePacketConstPtr& packet_msg);
  void ConfigCb(VelodynePuckConfig& config, int level);

 private:
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
  static_assert(sizeof(FiringSequence) == 48, "sizeof(FiringSequence) != 48");

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
    /// block. The time stamp’s value is the number of microseconds elapsed
    /// since the top of the hour.
    uint32_t stamp;
    uint8_t factory[2];
  } __attribute__((packed));
  static_assert(sizeof(Packet) == sizeof(velodyne_msgs::VelodynePacket().data),
                "sizeof(Packet) != 1206");

  /// Decoded result
  struct FiringSequenceStamped {
    int64_t time;
    float azimuth;  // rad [0, 2pi)
    FiringSequence sequence;
  };

  void DecodeAndFill(const Packet* const packet_buf);

  // TODO: use vector or array?
  using Decoded = std::array<FiringSequenceStamped, kFiringSequencesPerPacket>;
  Decoded DecodePacket(const Packet* packet, int64_t time) const;

  /// Convert firing sequences to image data
  sensor_msgs::ImagePtr ToImage(
      const std::vector<FiringSequenceStamped>& tfseqs,
      sensor_msgs::CameraInfo& cinfo) const;

  /// Publish
  void PublishBufferAndClear();
  void PublishCloud(const sensor_msgs::ImageConstPtr& image_msg,
                    const sensor_msgs::CameraInfoConstPtr& cinfo_msg);

 private:
  bool CheckFactoryBytes(const Packet* const packet);
  void Reset();

  // ROS related parameters
  std::string frame_id_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  ros::Subscriber packet_sub_;
  ros::Publisher cloud_pub_;
  image_transport::Publisher intensity_pub_, range_pub_;
  image_transport::CameraPublisher camera_pub_;
  dynamic_reconfigure::Server<VelodynePuckConfig> cfg_server_;
  VelodynePuckConfig config_;

  //
  cv::Mat image_;
  std::vector<double> azimuths_;
  std::vector<uint64_t> timestamps_;
  int curr_col_{0};

  std::vector<FiringSequenceStamped> buffer_;
};

/// Struct for precomputing sin and cos
struct SinCos {
  SinCos() = default;
  SinCos(float rad) : sin(std::sin(rad)), cos(std::cos(rad)) {}
  float sin, cos;
};

/// Convert image and camera_info to point cloud
CloudT ToCloud(const ImageConstPtr& image_msg, const CameraInfo& cinfo_msg,
               bool organized);

Decoder::Decoder(const ros::NodeHandle& pnh)
    : pnh_(pnh), it_(pnh), cfg_server_(pnh) {
  pnh_.param<std::string>("frame_id", frame_id_, "velodyne");
  ROS_INFO("Velodyne frame_id: %s", frame_id_.c_str());
  cfg_server_.setCallback(boost::bind(&Decoder::ConfigCb, this, _1, _2));
}

Decoder::Decoded Decoder::DecodePacket(const Packet* packet,
                                       int64_t time) const {
  // transform
  //               ^ x_l
  //               | -> /
  //               | a /
  //               |  /
  //               | /
  // <-------------o
  // y_l

  // std::array<TimedFiringSequence, kFiringSequencesPerPacket>;
  Decoded decoded;
  // For each data block, 12 total
  for (int dbi = 0; dbi < kDataBlocksPerPacket; ++dbi) {
    const auto& block = packet->blocks[dbi];
    const auto raw_azimuth = block.azimuth;
    ROS_WARN_STREAM_COND(raw_azimuth > kMaxRawAzimuth,
                         "Invalid raw azimuth: " << raw_azimuth);
    ROS_WARN_COND(block.flag != UPPER_BANK, "Invalid block %d", dbi);

    // Fill in decoded
    // for each firing sequence in the data block, 2
    for (int fsi = 0; fsi < kFiringSequencesPerDataBlock; ++fsi) {
      // Index into decoded, hardcode 2 for now
      const auto di = dbi * 2 + fsi;
      FiringSequenceStamped& tfseq = decoded[di];
      // Assume all firings within each firing sequence occur at the same time
      tfseq.time = time + di * kFiringCycleNs;
      tfseq.azimuth = Raw2Azimuth(block.azimuth);  // need to fix half later
      tfseq.sequence = block.sequences[fsi];
    }
  }

  // Fix azimuth for odd firing sequences
  for (int dbi = 0; dbi < kDataBlocksPerPacket; ++dbi) {
    // 1,3,5,...,23
    // Index into decoded with odd id, hardcode 2 for now
    const auto di = dbi * 2 + 1;
    auto azimuth = decoded[di].azimuth;

    auto prev = di - 1;
    auto next = di + 1;
    // Handle last block where there's no next to inerpolate
    // Just use the previous two
    if (dbi == kDataBlocksPerPacket - 1) {
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

bool Decoder::CheckFactoryBytes(const Packet* const packet_buf) {
  // Check return mode and product id, for now just die
  const auto return_mode = packet_buf->factory[0];
  if (!(return_mode == 55 || return_mode == 56)) {
    ROS_ERROR(
        "return mode must be Strongest (55) or Last Return (56), "
        "instead got (%u)",
        return_mode);
    return false;
  }
  const auto product_id = packet_buf->factory[1];
  if (product_id != 34) {
    ROS_ERROR("product id must be VLP-16 or Puck Lite (34), instead got (%u)",
              product_id);
    return false;
  }
  return true;
}

void Decoder::DecodeAndFill(const Packet* const packet_buf) {
  if (!CheckFactoryBytes(packet_buf)) {
    ros::shutdown();
  }

  // transform
  //              ^ x_l
  //              | -> /
  //              | a /
  //              |  /
  //              | /
  // <------------o
  // y_l

  // For each data block, 12 total
  for (int dbi = 0; dbi < kDataBlocksPerPacket; ++dbi) {
    const auto& block = packet_buf->blocks[dbi];
    auto raw_azimuth = block.azimuth;         // nominal azimuth [0,35999]
    auto azimuth = Raw2Azimuth(raw_azimuth);  // nominal azimuth [0, 2pi)

    ROS_WARN_STREAM_COND(raw_azimuth > kMaxRawAzimuth,
                         "Invalid raw azimuth: " << raw_azimuth);
    ROS_WARN_COND(block.flag != UPPER_BANK, "Invalid block %d", dbi);

    // First, adjust for an azimuth rollover from 359.99 to 0

    // for each firing sequence in the data block, 2
    for (int fsi = 0; fsi < kFiringSequencesPerDataBlock; ++fsi, ++curr_col_) {
      const auto col = dbi * 2 + fsi;
      // 9.5 Precision Azimuth Calculation
      // First, adjust for an Azimuth rollover from 359.99 to 0
      // for each laser beam, 16
      for (int lid = 0; lid < kFiringsPerFiringSequence; ++lid) {
      }

      timestamps_[curr_col_] = col * kFiringCycleNs;
      azimuths_[curr_col_] = Raw2Azimuth(block.azimuth);
    }
  }
}

void Decoder::PacketCb(const VelodynePacketConstPtr& packet_msg) {
  const auto start = ros::Time::now();

  const auto* packet = reinterpret_cast<const Packet*>(&(packet_msg->data[0]));

  // TODO: maybe make this a vector? to handle invalid data block?
  const auto decoded = DecodePacket(packet, packet_msg->stamp.toNSec());

  for (const auto& tfseq : decoded) {
    buffer_.push_back(tfseq);
    if (buffer_.size() >= static_cast<size_t>(config_.image_width)) {
      ROS_DEBUG("Publish fixed width with buffer size: %zu, required: %d",
                buffer_.size(), config_.image_width);
      PublishBufferAndClear();
    }
  }

  ROS_DEBUG("Time: %f", (ros::Time::now() - start).toSec());
}

void Decoder::ConfigCb(VelodynePuckConfig& config, int level) {
  config.min_range = std::min(config.min_range, config.max_range);

  ROS_INFO(
      "Reconfigure Request: min_range: %f, max_range: %f, image_width: %d, "
      "organized: %s, full_sweep: %s",
      config.min_range, config.max_range, config.image_width,
      config.organized ? "True" : "False",
      config.full_sweep ? "True" : "False");

  if (config.full_sweep) {
    ROS_WARN("Not supported for now.");
  } else {
    config.image_width /= kFiringSequencesPerPacket;
    config.image_width *= kFiringSequencesPerPacket;
  }

  config_ = config;
  buffer_.clear();
  Reset();

  if (level < 0) {
    ROS_INFO("Initialize ROS subscriber/publisher...");
    camera_pub_ = it_.advertiseCamera("image", 10);
    cloud_pub_ = pnh_.advertise<PointCloud2>("cloud", 10);
    intensity_pub_ = it_.advertise("intensity", 1);
    range_pub_ = it_.advertise("range", 1);

    packet_sub_ =
        pnh_.subscribe<VelodynePacket>("packet", 256, &Decoder::PacketCb, this);
    ROS_INFO("Decoder initialized");
  }
}

void Decoder::PublishBufferAndClear() {
  if (buffer_.empty()) return;

  // Always convert to image data
  const CameraInfoPtr cinfo_msg(new CameraInfo);
  const auto image_msg = ToImage(buffer_, *cinfo_msg);

  if (camera_pub_.getNumSubscribers() > 0) {
    camera_pub_.publish(image_msg, cinfo_msg);
  }

  if (cloud_pub_.getNumSubscribers() > 0) {
    PublishCloud(image_msg, cinfo_msg);
  }

  ROS_DEBUG("Clearing buffer %zu", buffer_.size());
  buffer_.clear();
}

void Decoder::PublishCloud(const ImageConstPtr& image_msg,
                           const CameraInfoConstPtr& cinfo_msg) {
  const auto cloud = ToCloud(image_msg, *cinfo_msg, config_.organized);
  ROS_DEBUG("number of points in cloud: %zu", cloud.size());
  cloud_pub_.publish(cloud);
}

void Decoder::Reset() {
  curr_col_ = 0;
  image_ = cv::Mat(kFiringsPerFiringSequence, config_.image_width, CV_32FC3,
                   cv::Scalar(kNaNF));
  azimuths_.clear();
  azimuths_.resize(config_.image_width, kNaND);
  timestamps_.clear();
  timestamps_.resize(config_.image_width, 0);
}

ImagePtr Decoder::ToImage(const std::vector<FiringSequenceStamped>& fseqs,
                          CameraInfo& cinfo_msg) const {
  std_msgs::Header header;
  header.stamp.fromNSec(fseqs.front().time);
  header.frame_id = frame_id_;

  cv::Mat image = cv::Mat::zeros(kFiringsPerFiringSequence, fseqs.size(),
                                 CV_32FC(kChannels));
  ROS_DEBUG("image: %d x %d x %d", image.rows, image.cols, image.channels());

  cinfo_msg.header = header;
  cinfo_msg.height = image.rows;
  cinfo_msg.width = image.cols;
  cinfo_msg.K[0] = kMinElevation;
  cinfo_msg.K[1] = kMaxElevation;
  cinfo_msg.R[0] = kDistanceResolution;
  cinfo_msg.P[0] = kFiringCycleNs;   // ns
  cinfo_msg.P[1] = kSingleFiringNs;  // ns
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
      const auto rr = Index2LaserId(kFiringsPerFiringSequence - 1 - r);

      // We clip range in image instead of in cloud
      float range = tfseq.sequence.points[rr].distance * kDistanceResolution;
      if (range < config_.min_range || range > config_.max_range) {
        range = kNaNF;
      }

      auto& e = image.at<Veckf>(r, c);
      e[0] = range;                                   // range
      e[1] = tfseq.sequence.points[rr].reflectivity;  // intensity
    }
  }

  return cv_bridge::CvImage(header, "32FC" + std::to_string(kChannels), image)
      .toImageMsg();
}

CloudT ToCloud(const ImageConstPtr& image_msg, const CameraInfo& cinfo_msg,
               bool organized) {
  CloudT cloud;

  const auto image = cv_bridge::toCvShare(image_msg)->image;
  const auto& azimuths = cinfo_msg.D;

  const float min_elevation = cinfo_msg.K[0];
  const float max_elevation = cinfo_msg.K[1];
  const float delta_elevation =
      (max_elevation - min_elevation) / (image.rows - 1);

  // Precompute sin cos
  std::vector<SinCos> sincos;
  sincos.reserve(azimuths.size());
  for (const auto& a : azimuths) {
    sincos.emplace_back(a);
  }

  cloud.header = pcl_conversions::toPCL(image_msg->header);
  cloud.reserve(image.total());

  for (int r = 0; r < image.rows; ++r) {
    const auto* const row_ptr = image.ptr<Veckf>(r);
    // Because image row 0 is the highest laser point
    const float omega = max_elevation - r * delta_elevation;
    const auto cos_omega = std::cos(omega);
    const auto sin_omega = std::sin(omega);

    for (int c = 0; c < image.cols; ++c) {
      const Veckf& data = row_ptr[c];

      PointT p;
      if (std::isnan(data[0])) {
        if (organized) {
          p.x = p.y = p.z = kNaNF;
          cloud.points.push_back(p);
        }
      } else {
        // p.53 Figure 9-1 VLP-16 Sensor Coordinate System
        // x = d * cos(w) * sin(a);
        // y = d * cos(w) * cos(a);
        // z = d * sin(w)
        const float R = data[RANGE];
        const auto x = R * cos_omega * sincos[c].cos;
        const auto y = R * cos_omega * sincos[c].sin;
        const auto z = R * sin_omega;

        p.x = x;
        p.y = -y;
        p.z = z;
        p.intensity = data[INTENSITY];

        cloud.points.push_back(p);
      }
    }
  }

  if (organized) {
    cloud.width = image.cols;
    cloud.height = image.rows;
  } else {
    cloud.width = cloud.size();
    cloud.height = 1;
  }

  return cloud;
}

}  // namespace velodyne_puck

int main(int argc, char** argv) {
  ros::init(argc, argv, "velodyne_puck_decoder");
  ros::NodeHandle pnh("~");

  velodyne_puck::Decoder node(pnh);
  ros::spin();
}
