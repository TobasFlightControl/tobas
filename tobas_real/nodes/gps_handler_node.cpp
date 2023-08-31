#include "../include/tobas_real/gps_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real::GpsHandler node(nh, pnh);
  node.run();
}
