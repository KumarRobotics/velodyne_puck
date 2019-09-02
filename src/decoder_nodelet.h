#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include "decoder.h"

namespace velodyne_puck {

class DecoderNodelet : public nodelet::Nodelet {
 public:
  DecoderNodelet() {}
  ~DecoderNodelet() {}

 private:
  virtual void onInit();
  Decoder::Ptr decoder;
};

}  // namespace velodyne_puck
