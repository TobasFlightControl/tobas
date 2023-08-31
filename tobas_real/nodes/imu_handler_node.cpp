#include "../include/tobas_real/imu_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real::ImuHandler node(nh, pnh);
  ros::spin();
}
