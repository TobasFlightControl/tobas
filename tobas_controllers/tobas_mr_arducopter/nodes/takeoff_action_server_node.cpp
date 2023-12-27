#include "../include/tobas_mr_arducopter/takeoff_action_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "takeoff_action_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_mr_arducopter::TakeoffActionServer node(nh, pnh);
  ros::spin();
}
