#include "../../include/tobas_real/cpu_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cpu_handler");
  tobas_real::CpuHandler node;
  node.run();
}
