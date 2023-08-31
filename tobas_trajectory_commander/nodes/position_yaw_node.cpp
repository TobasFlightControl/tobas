#include "../include/tobas_trajectory_commander/position_yaw.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_position_yaw_trajectory_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_trajectory_commander::FollowPositionYawTrajectoryServer node(nh, pnh);
  ros::spin();
}
