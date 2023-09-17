#include "../include/tobas_mr_lqrmpc/controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_mr_lqrmpc");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_mr_lqrmpc::ControllerRos node(nh, pnh);
  ros::spin();
}
