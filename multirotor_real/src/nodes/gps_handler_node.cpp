#include "../../include/multirotor_real/gps_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_handler");
  ros::NodeHandle nh;
  GpsHandler node(nh);
  ros::spin();
}
