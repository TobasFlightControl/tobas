#include "../include/tobas_arducopter_takeoff/takeoff_action_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "takeoff_action_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_arducopter_takeoff::TakeoffActionServer node(nh, pnh);
  ros::spin();
}
