#include "../include/tobas_gazebo_plugins/common.hpp"

namespace gazebo
{
ros::TransportHints tcpNoDelay(bool nodelay)
{
  return ros::TransportHints().reliable().tcpNoDelay(nodelay);
}
}  // namespace gazebo
