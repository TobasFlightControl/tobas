#include "../include/tobas_mavros_bridge/bridge.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_mr_arducopter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_mavros_bridge::TobasMavrosBridge node(nh, pnh);
  ros::spin();
}
