#include "../include/tobas_navio_ros/barometer_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navio_barometer_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_navio_ros::BarometerHandler node(nh, pnh);
  ros::spin();
}
