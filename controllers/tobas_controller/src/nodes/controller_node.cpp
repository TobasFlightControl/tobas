#include "../../include/tobas_controller/controller.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_controller");
  Controller node;
  ros::spin();
}
