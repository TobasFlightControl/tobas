#include "../../include/multirotor_real/imu_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_handler");
  ImuHandler node;
  ros::spin();
}
