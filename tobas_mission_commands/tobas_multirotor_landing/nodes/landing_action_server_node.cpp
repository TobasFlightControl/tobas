#include "../include/tobas_multirotor_landing/landing_action_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "landing_action_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  tobas_multirotor_landing::LandActionServer node(nh, pnh);
  ros::spin();
}
