#include "../include/tobas_real/pwm_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pwm_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real::PwmHandler node(nh, pnh);
  ros::spin();
}
