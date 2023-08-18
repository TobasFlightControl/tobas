#include "../../include/tobas_real/imu_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_handler");
  tobas_real::ImuHandler node;
  node.run();
}
