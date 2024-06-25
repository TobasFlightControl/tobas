#include "../include/tobas_topic_throttle/euler_throttle.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "euler_throttle");
  ros::NodeHandle nh;
  tobas_topic_throttle::EulerThrottle node(nh);
  ros::spin();
}
