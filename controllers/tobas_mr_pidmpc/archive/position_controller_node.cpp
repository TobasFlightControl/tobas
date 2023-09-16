#include "../include/tobas_mr_pidmpc/position_controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_mr_pidmpc::PositionControllerRos node(nh, pnh);
  ros::spin();
}
