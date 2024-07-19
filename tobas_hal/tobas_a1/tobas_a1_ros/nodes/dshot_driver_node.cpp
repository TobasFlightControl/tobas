#include "../include/tobas_a1_ros/dshot_driver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "a1_dshot_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  a1::DShotDriver node(nh, pnh);
  ros::spin();
}
