#include "../include/tobas_mr_pidmpc/controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_mr_pidmpc");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_mr_pidmpc::ControllerRos node(nh, pnh);
  ros::spin();
}
