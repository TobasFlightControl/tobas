#include "../include/tobas_multirotor_move/move_action_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_action_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_multirotor_move::MultirotorMoveServer node(nh, pnh);
  ros::spin();
}
