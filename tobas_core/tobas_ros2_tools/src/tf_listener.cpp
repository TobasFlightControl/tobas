#include "tobas_ros2_tools/tf_listener.hpp"

using namespace std;

namespace ros2
{
TransformListener::TransformListener(rclcpp::Node::SharedPtr node)
  : tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_, node)
{
}

bool TransformListener::lookupTransform(const string& parent, const string& child, const rclcpp::Time& time)
{
  try {
    tf_ = tf_buffer_.lookupTransform(parent, child, time);
  }
  catch (tf2::TransformException& e) {
    error_msg_ = e.what();
    return false;
  }

  return true;
}
}  // namespace ros2
