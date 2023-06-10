#include "../../include/tobas_real/motors_handler_dshot.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motors_handler_dshot");
  tobas_real::MotorsHandler_DSHOT node;
  node.run();
}
