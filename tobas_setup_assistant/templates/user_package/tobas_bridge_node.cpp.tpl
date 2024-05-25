#include "../include/{{ user_pkg_name }}/tobas_bridge.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tobas_bridge");
  const ros::NodeHandle nh;
  const ros::NodeHandle pnh;
  TobasBridge node(nh, pnh);
  ros::spin();
}
