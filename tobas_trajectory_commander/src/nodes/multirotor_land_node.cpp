#include "../../include/tobas_trajectory_commander/multirotor_land.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multirotor_land_server");
  tobas_trajectory_commander::MultirotorLandServer node;
  ros::spin();
}
