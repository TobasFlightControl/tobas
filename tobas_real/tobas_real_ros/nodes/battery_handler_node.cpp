#include "../include/tobas_real_ros/battery_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real_ros::BatteryHandler node(nh, pnh);
  ros::spin();
}
