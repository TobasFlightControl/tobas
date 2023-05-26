#include "../../include/tobas_fixed_wing_lqr/controller.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_fixed_wing_lqr");
  tobas_fixed_wing_lqr::Controller node;
  ros::spin();
}
