#include "../include/tobas_a1_ros/imu_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "a1_imu_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  a1::IMUHandler node(nh, pnh);
  ros::spin();
}
