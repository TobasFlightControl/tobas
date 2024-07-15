#include "../include/tobas_real_ros/imu_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real_ros::ImuHandler node(nh, pnh);
  ros::spin();
}
