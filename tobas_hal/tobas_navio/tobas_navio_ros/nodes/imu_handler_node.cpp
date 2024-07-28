#include "../include/tobas_navio_ros/imu_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navio_imu_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_navio_ros::ImuHandler node(nh, pnh);
  ros::spin();
}
