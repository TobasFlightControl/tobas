#include "../include/tobas_latency_publisher/latency_publisher.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "latency_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_latency_publisher::LatencyPublisher node(nh, pnh);
  ros::spin();
}
