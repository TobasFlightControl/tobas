#include "../include/tobas_navio_ros/pwm_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navio_pwm_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_navio_ros::PwmHandler node(nh, pnh);
  ros::spin();
}
