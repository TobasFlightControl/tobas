#include "../include/tobas_ros_tools/tf_listener.hpp"

using namespace std;

namespace tobas_ros
{
TransformListener::TransformListener() : tf_listener_(tf_buffer_)
{
}

bool TransformListener::lookupTransform(
  const string& parent,
  const string& child,
  const ros::Time& time)
{
  try
  {
    tf_ = tf_buffer_.lookupTransform(parent, child, time);
  }
  catch (tf2::TransformException& e)
  {
    error_msg_ = e.what();
    return false;
  }

  return true;
}
}  // namespace tobas_ros
