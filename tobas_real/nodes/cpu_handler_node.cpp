#include "../include/tobas_real/cpu_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cpu_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real::CpuHandler node(nh, pnh);
  ros::spin();
}
