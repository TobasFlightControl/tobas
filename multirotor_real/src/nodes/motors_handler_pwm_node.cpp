#include "../../include/multirotor_real/motors_handler_pwm.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motors_handler_pwm");
  ros::NodeHandle nh;
  MotorsHandler_PWM node(nh);
  ros::spin();
}
