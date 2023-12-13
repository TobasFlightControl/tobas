#include "../include/tobas_manipulation/velocity_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_space_controller_velocity");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_manipulation::VelocityControllerRos node(nh, pnh);
  ros::spin();
}
