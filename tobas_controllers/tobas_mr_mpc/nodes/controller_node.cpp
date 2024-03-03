#include "../include/tobas_mr_mpc/controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_mr_mpc");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_mr_mpc::ControllerRos node(nh, pnh);
  ros::spin();
}
