#include "../../include/tobas_state_checker/multirotor_state_checker.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multirotor_state_checker");
  tobas_state_checker::MultirotorStateChecker node;
  node.run();
}
