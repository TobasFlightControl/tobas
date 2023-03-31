#include "../../include/multirotor_controller/controller.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multirotor_controller");
  Controller node;
  ros::spin();
}
