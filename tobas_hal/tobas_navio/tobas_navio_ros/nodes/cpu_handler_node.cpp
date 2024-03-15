#include "../include/tobas_navio_ros/cpu_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cpu_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_navio_ros::CpuHandler node(nh, pnh);
  ros::spin();
}
