#include "../../include/tobas_real/battery_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_handler");
  tobas_real::BatteryHandler node;
  ros::spin();
}
