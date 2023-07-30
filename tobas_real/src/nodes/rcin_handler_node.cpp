#include "../../include/tobas_real/rcin_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rc_input_handler");
  tobas_real::RCInputHandler node;
  node.run();
}
