#include <pluginlib/class_list_macros.h>

#include "decoder_nodelet.h"

namespace velodyne_puck {

void DecoderNodelet::onInit() {
  decoder.reset(new Decoder(getPrivateNodeHandle()));
}

}  // namespace velodyne_puck

PLUGINLIB_EXPORT_CLASS(velodyne_puck::DecoderNodelet, nodelet::Nodelet);
