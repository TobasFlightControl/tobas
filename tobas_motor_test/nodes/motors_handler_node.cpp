#include "../include/tobas_motor_test/motors_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motors_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_motor_test::MotorsHandler node(nh, pnh);
  ros::spin();
}
