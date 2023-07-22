#include "../../include/static_state_determination/static_state_determination.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_state_determination");
  static_state_determination::StaticStateDeterminationServer node;
  ros::spin();
}
