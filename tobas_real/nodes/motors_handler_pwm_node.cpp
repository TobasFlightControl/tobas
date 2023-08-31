#include "../include/tobas_real/motors_handler_pwm.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motors_handler_pwm");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real::MotorsHandler_PWM node(nh, pnh);
  node.run();
}
