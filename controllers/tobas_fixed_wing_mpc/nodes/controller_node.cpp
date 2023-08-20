#include "../include/tobas_fixed_wing_mpc/controller.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_fixed_wing_mpc");
  tobas_fixed_wing_mpc::Controller node;
  ros::spin();
}
