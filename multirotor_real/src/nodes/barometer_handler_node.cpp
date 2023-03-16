#include "../../include/multirotor_real/barometer_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barometer_handler");
  ros::NodeHandle nh;
  BarometerHandler node(nh);
  ros::spin();
}
