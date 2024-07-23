#include "../include/tobas_a1_ros/adc_driver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "a1_adc_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  a1::ADCDriver node(nh, pnh);
  ros::spin();
}
