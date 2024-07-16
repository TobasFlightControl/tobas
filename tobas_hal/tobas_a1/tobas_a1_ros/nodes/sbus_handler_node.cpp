#include "../include/tobas_a1_ros/sbus_handler.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "a1_sbus_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_a1_ros::SBUSHandler node(nh, pnh);
  ros::spin();
}
