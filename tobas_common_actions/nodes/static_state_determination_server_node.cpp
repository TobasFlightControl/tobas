#include "../include/tobas_common_actions/static_state_determination_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_state_determination_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_common_actions::StaticStateDeterminationServer node(nh, pnh);
  ros::spin();
}
