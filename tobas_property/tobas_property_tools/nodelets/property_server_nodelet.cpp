#include <pluginlib/class_list_macros.hpp>

#include "./property_server_nodelet.hpp"

namespace ptree
{
void PropertyServerNodelet::onInit()
{
  node_.reset(new PropertyServer(getNodeHandle(), getPrivateNodeHandle(), getName()));
}
}  // namespace ptree

PLUGINLIB_EXPORT_CLASS(ptree::PropertyServerNodelet, nodelet::Nodelet);
