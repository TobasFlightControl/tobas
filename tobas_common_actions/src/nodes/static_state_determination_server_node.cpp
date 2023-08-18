#include "../../include/tobas_common_actions/static_state_determination_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_state_determination_server");
  tobas_common_actions::StaticStateDeterminationServer node;
  ros::spin();
}
