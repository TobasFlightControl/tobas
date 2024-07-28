#include "../include/tobas_a1_ros/pwm_driver.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "a1_pwm_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  a1::PWMDriver node(nh, pnh);
  ros::spin();
}
