#include "../include/tobas_mr_pidmpc/velocity_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_mr_pidmpc::VelocityControllerRos node(nh, pnh);
  ros::spin();
}
