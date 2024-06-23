#include "../include/tobas_property_tools/property_server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "property_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ptree::PropertyServer node(nh, pnh);
  ros::spin();
}
