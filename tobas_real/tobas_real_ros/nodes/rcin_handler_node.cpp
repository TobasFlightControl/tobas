#include "../include/tobas_real_ros/rcin_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rcin_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real_ros::RCInputHandler node(nh, pnh);
  ros::spin();
}
