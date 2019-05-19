#pragma once

#include <cmath>
#include <cstdint>

namespace velodyne_puck_decoder {

static constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }
static constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }

/// All 16 lasers are fired and recharged every 55.296 us.
/// The time between firings is 2.304 us.
/// There are 16 firings (in each sequence) followed by an idel period of
/// 18.43 us.
/// Therefore, the timing cycle to fire and recharge all 16 lasers is given by
/// (16 * 2.304us) + 18.432us = 55.296us
///
/// Firing Sequence
/// A firing sequence occurs when all the lasers in a sensor are fired. They are
/// fired in a sequence specific to a given product line.
///
/// Data Point
/// A data point is represented in the packet by three bytes - 2 for distance
/// and 1 for calibrated reflectivity. The distance is an unsigned integer. It
/// has 2mm granulairty. Calibrated reflectivity is reported on a scale of 0 to
/// 255. d(m) = d(raw) / 500.
///
/// Azimuth
/// A 2-byte azimuth value (alpha) appears after the flag bytes at the beginning
/// of each data block. It is an unsigned integer which represents an angle in
/// hundredths of a degree. alpha(deg) = alpha(raw) / 100.0.
///
/// Data Block
/// The information from 2 firing sequences of 16 lasres is contained in each
/// data block. Each packet contains the data from 24 firing sequences in 12
/// data blocks. A data block consits of 100 bytes of binary data.
/// For calculating time offsets it is recommended that the data blocks in a
/// packet be numbered from 0 to 11.

// Raw Velodyne packet constants and structures.
static constexpr int kPointBytes = 3;
static constexpr int kPointsPerBlock = 32;
static constexpr int kPointBytesPerBlock =
    (kPointBytes * kPointsPerBlock);  // 96

// According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
// valid packets with readings up to 130.0.
static const double kDistanceMax = 130.0;         // [m]
static const double kDistanceResolution = 0.002;  // [m]

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static constexpr double kSingleFiringUs = 2.304;  // [µs]
static constexpr double kFiringCycleUs = 55.296;  // [µs]

// The information from two firing sequences of 16 lasers is contained in each
// data block. Each packet contains the data from 24 firing sequences in 12 data
// blocks.
static constexpr int kFiringsPerCycle = 16;
static constexpr int kFiringsPerBlock = 2;
static constexpr int kBlocksPerPacket = 12;
static constexpr int kFiringsPerPacket =
    kFiringsPerBlock * kBlocksPerPacket;  // 24

inline constexpr int LaserId2Index(int id) {
  return id % 2 == 0 ? id / 2 : id / 2 + kFiringsPerCycle / 2;
}

// Pre-compute the sine and cosine for the altitude angles.
static constexpr float kMinElevation = deg2rad(-15.0);
static constexpr float kMaxElevation = deg2rad(15.0);
static constexpr float kDeltaElevation =
    (kMaxElevation - kMinElevation) / (kFiringsPerCycle - 1);

inline constexpr float RawAzimuthToFloat(uint16_t raw_azimuth) {
  // According to the user manual,
  return deg2rad(static_cast<float>(raw_azimuth) / 100.0);
}

/// p51 8.3.1
inline constexpr float AzimuthResolutionDegree(int rpm) {
  // rpm % 60 == 0
  return rpm / 60.0 * 360.0 * kFiringCycleUs / 1e6;
}

}  // namespace velodyne_puck_decoder
