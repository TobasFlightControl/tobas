#include "../include/tobas_real_ros/barometer_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barometer_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_real_ros::BarometerHandler node(nh, pnh);
  ros::spin();
}
