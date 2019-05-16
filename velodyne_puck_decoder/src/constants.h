#pragma once

#include <cmath>
#include <cstdint>

namespace velodyne_puck_decoder {

// static constexpr double DEG_TO_RAD = 0.017453292;
// static constexpr double RAD_TO_DEG = 57.29577951;

static constexpr double deg2rad(double deg) { return deg * M_PI / 180.0; }
static constexpr double rad2deg(double rad) { return rad * 180.0 / M_PI; }

// Raw Velodyne packet constants and structures.
static constexpr int SIZE_BLOCK = 100;
static constexpr int RAW_SCAN_SIZE = 3;
static constexpr int SCANS_PER_BLOCK = 32;
static constexpr int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

// According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
// valid packets with readings up to 130.0.
static const double DISTANCE_MAX = 130.0;        /**< meters */
static const double DISTANCE_RESOLUTION = 0.002; /**< meters */
static const double DISTANCE_MAX_UNITS =
    (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0);

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int FIRINGS_PER_BLOCK = 2;
static const int SCANS_PER_FIRING = 16;
static const double BLOCK_TDURATION = 110.592;  // [µs]
static const double DSR_TOFFSET = 2.304;        // [µs]
static const double kFiringCycleUs = 55.296;    // [µs]

static const int PACKET_SIZE = 1206;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static constexpr int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
static constexpr int FIRINGS_PER_PACKET = FIRINGS_PER_BLOCK * BLOCKS_PER_PACKET;

// Pre-compute the sine and cosine for the altitude angles.
static const double scan_altitude[16] = {
    -0.2617993877991494,  0.017453292519943295, -0.22689280275926285,
    0.05235987755982989,  -0.19198621771937624, 0.08726646259971647,
    -0.15707963267948966, 0.12217304763960307,  -0.12217304763960307,
    0.15707963267948966,  -0.08726646259971647, 0.19198621771937624,
    -0.05235987755982989, 0.22689280275926285,  -0.017453292519943295,
    0.2617993877991494};

static constexpr double cos_scan_altitude[16] = {
    std::cos(scan_altitude[0]),  std::cos(scan_altitude[1]),
    std::cos(scan_altitude[2]),  std::cos(scan_altitude[3]),
    std::cos(scan_altitude[4]),  std::cos(scan_altitude[5]),
    std::cos(scan_altitude[6]),  std::cos(scan_altitude[7]),
    std::cos(scan_altitude[8]),  std::cos(scan_altitude[9]),
    std::cos(scan_altitude[10]), std::cos(scan_altitude[11]),
    std::cos(scan_altitude[12]), std::cos(scan_altitude[13]),
    std::cos(scan_altitude[14]), std::cos(scan_altitude[15]),
};

static constexpr double sin_scan_altitude[16] = {
    std::sin(scan_altitude[0]),  std::sin(scan_altitude[1]),
    std::sin(scan_altitude[2]),  std::sin(scan_altitude[3]),
    std::sin(scan_altitude[4]),  std::sin(scan_altitude[5]),
    std::sin(scan_altitude[6]),  std::sin(scan_altitude[7]),
    std::sin(scan_altitude[8]),  std::sin(scan_altitude[9]),
    std::sin(scan_altitude[10]), std::sin(scan_altitude[11]),
    std::sin(scan_altitude[12]), std::sin(scan_altitude[13]),
    std::sin(scan_altitude[14]), std::sin(scan_altitude[15]),
};

inline constexpr double rawAzimuthToDouble(uint16_t raw_azimuth) {
  // According to the user manual,
  // azimuth = raw_azimuth / 100.0;
  //    return static_cast<double>(raw_azimuth) / 100.0 * DEG_TO_RAD;
  return deg2rad(static_cast<double>(raw_azimuth) / 100.0);
}

}  // namespace velodyne_puck_decoder
