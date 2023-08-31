#include "../include/tobas_real/barometer_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barometer_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real::BarometerHandler node(nh, pnh);
  node.run();
}
