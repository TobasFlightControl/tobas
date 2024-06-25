#include "../include/tobas_preprocess/imu_lpf.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_lpf");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_preprocess::ImuLpf node(nh, pnh);
  ros::spin();
}
