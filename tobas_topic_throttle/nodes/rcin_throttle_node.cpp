#include "../include/tobas_topic_throttle/rcin_throttle.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rcin_throttle");
  ros::NodeHandle nh;
  tobas_topic_throttle::RCInputThrottle node(nh);
  ros::spin();
}
