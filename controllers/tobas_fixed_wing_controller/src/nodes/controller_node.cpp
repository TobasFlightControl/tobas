#include "../../include/tobas_fixed_wing_controller/controller_ros.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_fixed_wing_controller");
  tobas_fixed_wing_controller::Controller node;
  ros::spin();
}
