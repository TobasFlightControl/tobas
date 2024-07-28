#include "../include/tobas_a1_ros/mag_driver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "a1_mag_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  a1::MagDriver node(nh, pnh);
  ros::spin();
}
