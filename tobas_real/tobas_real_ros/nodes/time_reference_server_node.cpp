#include "../include/tobas_real_ros/time_reference_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "time_reference_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real::TimeReferenceServer node(nh, pnh);
  ros::spin();
}
