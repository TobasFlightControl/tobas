#include "../include/tobas_topic_throttle/latency_throttle.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "latency_throttle");
  ros::NodeHandle nh;
  tobas_topic_throttle::LatencyThrottle node(nh);
  ros::spin();
}
