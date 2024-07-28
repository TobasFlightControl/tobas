#include "../include/tobas_a1_ros/baro_driver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "a1_baro_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  a1::BaroDriver node(nh, pnh);
  ros::spin();
}
