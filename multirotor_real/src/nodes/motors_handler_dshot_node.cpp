#include "../../include/multirotor_real/motors_handler_dshot.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motors_handler_dshot");
  ros::NodeHandle nh;
  MotorsHandler_DSHOT node(nh);
  ros::spin();
}
