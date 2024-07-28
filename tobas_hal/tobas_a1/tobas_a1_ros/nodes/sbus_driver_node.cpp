#include "../include/tobas_a1_ros/sbus_driver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "a1_sbus_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  a1::SBUSDriver node(nh, pnh);
  ros::spin();
}
