#include "../include/tobas_navio_ros/motors_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motors_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_navio_ros::MotorsHandler node(nh, pnh);
  ros::spin();
}
