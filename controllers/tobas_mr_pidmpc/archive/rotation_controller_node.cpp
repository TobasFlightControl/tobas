#include "../include/tobas_mr_pidmpc/rotation_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotation_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_mr_pidmpc::RotationControllerRos node(nh, pnh);
  ros::spin();
}
