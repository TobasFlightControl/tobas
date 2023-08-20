#include "../include/tobas_fixed_wing_lqd/controller.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_fixed_wing_lqd");
  tobas_fixed_wing_lqd::Controller node;
  ros::spin();
}
