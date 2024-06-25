#include "../include/tobas_topic_throttle/battery_lpf_throttle.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_lpf_throttle");
  ros::NodeHandle nh;
  tobas_topic_throttle::BatteryLPFThrottle node(nh);
  ros::spin();
}
