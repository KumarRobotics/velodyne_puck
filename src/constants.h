#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace velodyne_puck {

static constexpr auto kPclNaN = std::numeric_limits<float>::quiet_NaN();
static constexpr float kTau = M_PI * 2;
static constexpr float deg2rad(float deg) { return deg * M_PI / 180.0; }
static constexpr float rad2deg(float rad) { return rad * 180.0 / M_PI; }

// Raw Velodyne packet constants and structures.
static constexpr int kPointBytes = 3;
static constexpr int kPointsPerBlock = 32;

// According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
// valid packets with readings up to 130.0.
static constexpr float kDistanceMax = 130.0;         // [m]
static constexpr float kDistanceResolution = 0.002;  // [m]

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static constexpr double kSingleFiringUs = 2.304;  // [µs]
static constexpr double kFiringCycleUs = 55.296;  // [µs]

// The information from two firing sequences of 16 lasers is contained in each
// data block. Each packet contains the data from 24 firing sequences in 12 data
// blocks.
static constexpr int kFiringsPerFiringSequence = 16;
static constexpr int kFiringSequencesPerDataBlock = 2;
static constexpr int kDataBlocksPerPacket = 12;
static constexpr int kFiringSequencesPerPacket =
    kFiringSequencesPerDataBlock * kDataBlocksPerPacket;  // 24

inline int LaserId2Index(int id) {
  return (id % 2 == 0) ? id / 2 : id / 2 + kFiringsPerFiringSequence / 2;
}

inline int Index2LaserId(int index) {
  const auto half = kFiringsPerFiringSequence / 2;
  return (index < half) ? index * 2 : (index - half) * 2 + 1;
}

static constexpr uint16_t kMaxRawAzimuth = 35999;
static constexpr float kAzimuthResolution = 0.01;

static constexpr float kMinElevation = deg2rad(-15.0);
static constexpr float kMaxElevation = deg2rad(15.0);
static constexpr float kDeltaElevation =
    (kMaxElevation - kMinElevation) / (kFiringsPerFiringSequence - 1);

inline constexpr float Azimuth(uint16_t raw) {
  // According to the user manual,
  return deg2rad(static_cast<float>(raw) * kAzimuthResolution);
}

/// p55 9.3.1.2
inline constexpr float Distance(uint16_t raw) {
  return static_cast<float>(raw) * kDistanceResolution;
}

/// p51 8.3.1
inline constexpr float AzimuthResolutionDegree(int rpm) {
  // rpm % 60 == 0
  return rpm / 60.0 * 360.0 * kFiringCycleUs / 1e6;
}

}  // namespace velodyne_puck
