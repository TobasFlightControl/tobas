#include "../include/tobas_navio_ros/time_reference_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "time_reference_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_navio_ros::TimeReferenceServer node(nh, pnh);
  ros::spin();
}
