#include "../include/tobas_real/battery_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real::BatteryHandler node(nh, pnh);
  ros::spin();
}
