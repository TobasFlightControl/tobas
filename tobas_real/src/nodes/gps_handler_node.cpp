#include "../../include/tobas_real/gps_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_handler");
  tobas_real::GpsHandler node;
  node.run();
}
