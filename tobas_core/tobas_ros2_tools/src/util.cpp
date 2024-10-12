#include <tobas_math/linalg.hpp>

#include "../include/tobas_ros2_tools/util.hpp"

namespace ros2
{
double norm(const geometry_msgs::msg::Vector3& v)
{
  return math::norm(v.x, v.y, v.z);
}

bool isFieldSizeMatch(const sensor_msgs::msg::JointState& js)
{
  const auto size = js.name.size();
  return js.position.size() == size && js.velocity.size() == size && js.effort.size() == size;
}

void clear(sensor_msgs::msg::JointState& js)
{
  js.name.clear();
  js.position.clear();
  js.velocity.clear();
  js.effort.clear();
}

void resize(sensor_msgs::msg::JointState& js, const size_t& size)
{
  js.name.resize(size);
  js.position.resize(size);
  js.velocity.resize(size);
  js.effort.resize(size);
}
}  // namespace ros2
