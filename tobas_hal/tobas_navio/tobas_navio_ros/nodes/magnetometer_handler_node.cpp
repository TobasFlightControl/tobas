#include "../include/tobas_navio_ros/magnetometer_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navio_magnetometer_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_navio_ros::MagnetometerHandler node(nh, pnh);
  ros::spin();
}
