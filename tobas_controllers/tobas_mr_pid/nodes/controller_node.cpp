#include "../include/tobas_mr_pid/controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_mr_pid");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_mr_pid::ControllerRos node(nh, pnh);
  ros::spin();
}
