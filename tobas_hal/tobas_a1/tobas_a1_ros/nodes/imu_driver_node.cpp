#include "../include/tobas_a1_ros/imu_driver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "a1_imu_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  a1::IMUDriver node(nh, pnh);
  ros::spin();
}
