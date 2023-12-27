#include "../include/tobas_np_pid/controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_np_pid");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_np_pid::ControllerRos node(nh, pnh);
  ros::spin();
}
