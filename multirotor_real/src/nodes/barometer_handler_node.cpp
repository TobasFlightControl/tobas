#include "../../include/multirotor_real/barometer_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barometer_handler");
  BarometerHandler node;
  ros::spin();
}
