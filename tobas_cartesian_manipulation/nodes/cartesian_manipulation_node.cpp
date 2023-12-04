#include "../include/tobas_cartesian_manipulation/cartesian_manipulation_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_manipulation");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_cartesian_manipulation::CartesianManipulationRos node(nh, pnh);
  ros::spin();
}
