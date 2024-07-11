#include "../include/tobas_navio_ros/rcin_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navio_rcin_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_navio_ros::RCInputHandler node(nh, pnh);
  ros::spin();
}
