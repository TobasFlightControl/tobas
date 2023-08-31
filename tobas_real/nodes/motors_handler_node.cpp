#include "../include/tobas_real/motors_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motors_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real::MotorsHandler node(nh, pnh);
  ros::spin();
}
