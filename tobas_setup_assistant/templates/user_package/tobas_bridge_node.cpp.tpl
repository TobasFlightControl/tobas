#include "../include/{{ user_pkg_name }}/tobas_bridge.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  TobasBridge node(nh, pnh);
  ros::spin();
}
