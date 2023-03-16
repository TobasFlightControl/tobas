#include "../../include/multirotor_real/motors_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motors_handler");
  ros::NodeHandle nh;
  MotorsHandler node(nh);
  ros::spin();
}
