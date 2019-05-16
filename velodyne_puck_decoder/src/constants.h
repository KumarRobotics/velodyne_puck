#pragma once

#include <cmath>
#include <cstdint>

namespace velodyne_puck_decoder {

static constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }
static constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }

// Raw Velodyne packet constants and structures.
static constexpr int kPointBytes = 3;
static constexpr int kPointsPerBlock = 32;
static constexpr int kPointBytesPerBlock = (kPointBytes * kPointsPerBlock);

// According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
// valid packets with readings up to 130.0.
static const double kDistanceMax = 130.0;        /**< meters */
static const double kDistanceResolution = 0.002; /**< meters */

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const double kSingleFiringUs = 2.304;  // [µs]
static const double kFiringCycleUs = 55.296;  // [µs]

// The information from two firing sequences of 16 lasers is contained in each
// data block. Each packet contains the data from 24 firing sequences in 12 data
// blocks.
static const int kFiringsPerCycle = 16;
static const int kFiringsPerBlock = 2;
static const int kBlocksPerPacket = 12;
static constexpr int kFiringsPerPacket =
    kFiringsPerBlock * kBlocksPerPacket;  // 24

inline constexpr int LaserId2Index(int id) {
  return id % 2 == 0 ? id / 2 : id / 2 + kFiringsPerCycle / 2;
}

// Pre-compute the sine and cosine for the altitude angles.
static const double kScanElevations[kFiringsPerCycle] = {
    -0.2617993877991494,  0.017453292519943295, -0.22689280275926285,
    0.05235987755982989,  -0.19198621771937624, 0.08726646259971647,
    -0.15707963267948966, 0.12217304763960307,  -0.12217304763960307,
    0.15707963267948966,  -0.08726646259971647, 0.19198621771937624,
    -0.05235987755982989, 0.22689280275926285,  -0.017453292519943295,
    0.2617993877991494};

static constexpr double kCosScanElevations[kFiringsPerCycle] = {
    std::cos(kScanElevations[0]),  std::cos(kScanElevations[1]),
    std::cos(kScanElevations[2]),  std::cos(kScanElevations[3]),
    std::cos(kScanElevations[4]),  std::cos(kScanElevations[5]),
    std::cos(kScanElevations[6]),  std::cos(kScanElevations[7]),
    std::cos(kScanElevations[8]),  std::cos(kScanElevations[9]),
    std::cos(kScanElevations[10]), std::cos(kScanElevations[11]),
    std::cos(kScanElevations[12]), std::cos(kScanElevations[13]),
    std::cos(kScanElevations[14]), std::cos(kScanElevations[15]),
};

static constexpr double kSinScanElevations[kFiringsPerCycle] = {
    std::sin(kScanElevations[0]),  std::sin(kScanElevations[1]),
    std::sin(kScanElevations[2]),  std::sin(kScanElevations[3]),
    std::sin(kScanElevations[4]),  std::sin(kScanElevations[5]),
    std::sin(kScanElevations[6]),  std::sin(kScanElevations[7]),
    std::sin(kScanElevations[8]),  std::sin(kScanElevations[9]),
    std::sin(kScanElevations[10]), std::sin(kScanElevations[11]),
    std::sin(kScanElevations[12]), std::sin(kScanElevations[13]),
    std::sin(kScanElevations[14]), std::sin(kScanElevations[15]),
};

double kCosAzimuthTable[6300];
double kSinAzimuthTable[6300];

inline constexpr double rawAzimuthToDouble(uint16_t raw_azimuth) {
  // According to the user manual,
  // azimuth = raw_azimuth / 100.0;
  //    return static_cast<double>(raw_azimuth) / 100.0 * DEG_TO_RAD;
  return deg2rad(static_cast<double>(raw_azimuth) / 100.0);
}

/// p51 8.3.1
inline double AzimuthResolutionDegree(int rpm) {
  // rpm % 60 == 0
  return rpm / 60.0 * 360.0 * kFiringCycleUs / 1e6;
}

}  // namespace velodyne_puck_decoder
