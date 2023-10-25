#include "../include/tobas_mr_arducopter/controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arducopter_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_mr_arducopter::ControllerRos node(nh, pnh);
  ros::spin();
}
