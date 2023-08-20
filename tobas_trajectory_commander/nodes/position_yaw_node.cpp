#include "../include/tobas_trajectory_commander/position_yaw.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "landing_action_server");
  tobas_trajectory_commander::FollowPositionYawTrajectoryServer node;
  ros::spin();
}
