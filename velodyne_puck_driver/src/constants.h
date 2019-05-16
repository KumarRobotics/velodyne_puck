#pragma once

namespace velodyne_puck_driver {

// p49 8.1
static constexpr double kFiringCycleUs = 55.296;
// p49 8.2.1
static constexpr int kFiringCyclePerPacket = 24;
static constexpr double kDelayPerPacketUs =
    kFiringCyclePerPacket * kFiringCycleUs;
static constexpr double kPacketsPerSecond = 1e6 / kDelayPerPacketUs;

}  // namespace velodyne_puck_driver
