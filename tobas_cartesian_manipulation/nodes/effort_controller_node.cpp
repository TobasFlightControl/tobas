#include "../include/tobas_cartesian_manipulation/effort_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartesian_controller_effort");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_cartesian_manipulation::EffortControllerRos node(nh, pnh);
  ros::spin();
}
