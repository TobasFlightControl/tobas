#include "../../include/tobas_multirotor_landing/landing_action_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "landing_action_server");
  tobas_multirotor_landing::MultirotorLandServer node;
  ros::spin();
}
