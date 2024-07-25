#include "../include/tobas_a1_ros/gnss_driver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "a1_gnss_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  a1::GNSSDriver node(nh, pnh);
  ros::spin();
}
