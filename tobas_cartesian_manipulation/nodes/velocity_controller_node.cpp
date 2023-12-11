#include "../include/tobas_cartesian_manipulation/velocity_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_controller_velocity");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_cartesian_manipulation::VelocityControllerRos node(nh, pnh);
  ros::spin();
}
