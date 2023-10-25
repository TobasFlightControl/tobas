#include "../include/tobas_real/rcin_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rc_input_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real::RCInputHandler node(nh, pnh);
  ros::spin();
}
