#include "../include/tobas_preprocess/battery_lpf.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_lpf");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_preprocess::BatteryLpf node(nh, pnh);
  ros::spin();
}
