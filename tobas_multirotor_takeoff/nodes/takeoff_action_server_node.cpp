#include "../include/tobas_multirotor_takeoff/takeoff_action_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "takeoff_action_server");
  tobas_multirotor_takeoff::MultirotorTakeoffServer node;
  ros::spin();
}
