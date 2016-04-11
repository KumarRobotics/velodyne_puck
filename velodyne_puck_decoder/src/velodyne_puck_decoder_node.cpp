#include <ros/ros.h>
//#include <google/profiler.h>
#include <velodyne_puck_decoder/velodyne_puck_decoder.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "velodyne_puck_decoder_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  velodyne_puck_decoder::VelodynePuckDecoderPtr decoder(
      new velodyne_puck_decoder::VelodynePuckDecoder(nh, pnh));

  if (!decoder->initialize()) {
    ROS_INFO("Cannot initialize the decoder...");
    return -1;
  }

  //ProfilerStart("velodyne_decoder.prof");
  ros::spin();
  //ProfilerStop();

  return 0;
}
