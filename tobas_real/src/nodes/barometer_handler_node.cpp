#include "../../include/tobas_real/barometer_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barometer_handler");
  tobas_real::BarometerHandler node;
  ros::spin();
}
