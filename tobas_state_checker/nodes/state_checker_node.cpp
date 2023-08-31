#include "../include/tobas_state_checker/state_checker.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_checker");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_state_checker::MultirotorStateChecker node(nh, pnh);
  node.run();
}
