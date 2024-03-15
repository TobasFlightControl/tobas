#include "../include/tobas_navio_ros/gps_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_navio_ros::GpsHandler node(nh, pnh);
  ros::spin();
}
