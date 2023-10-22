#include "../include/tobas_mr_arducopter/param_server_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arducopter_param_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_mr_arducopter::ParamServerRos node(nh, pnh);
  ros::spin();
}
