#include "../include/tobas_trajectory_commander/position_yaw.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "landing_action_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_trajectory_commander::FollowPositionYawTrajectoryServer node(nh, pnh);
  ros::spin();
}
