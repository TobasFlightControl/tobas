#include "../../include/tobas_multirotor_controller/controller.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_multirotor_controller");
  Controller node;
  ros::spin();
}
