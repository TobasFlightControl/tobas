#include "../include/tobas_topic_throttle/battery_throttle.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_throttle");
  ros::NodeHandle nh;
  tobas_topic_throttle::BatteryThrottle node(nh);
  ros::spin();
}
