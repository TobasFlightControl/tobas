#include "../include/tobas_manipulation/position_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_manipulation_position");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_manipulation::PositionControllerRos node(nh, pnh);
  ros::spin();
}
