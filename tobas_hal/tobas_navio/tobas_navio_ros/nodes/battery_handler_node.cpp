#include "../include/tobas_navio_ros/battery_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_navio_ros::BatteryHandler node(nh, pnh);
  ros::spin();
}
