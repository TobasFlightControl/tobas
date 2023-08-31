#include "../include/tobas_fixed_wing_lqd/controller.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_fixed_wing_lqd");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_fixed_wing_lqd::Controller node(nh, pnh);
  ros::spin();
}
