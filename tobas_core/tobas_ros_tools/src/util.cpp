#include <tobas_std_tools/math.hpp>

#include "../include/tobas_ros_tools/util.hpp"

using namespace tobas_std;

namespace tobas_ros
{
double norm(const geometry_msgs::Vector3& v)
{
  return sqrt(sqr(v.x) + sqr(v.y) + sqr(v.z));
}

bool isFieldSizeMatch(const sensor_msgs::JointState& js)
{
  const auto size = js.name.size();
  return js.position.size() == size && js.velocity.size() == size && js.effort.size() == size;
}

void clear(sensor_msgs::JointState& js)
{
  js.name.clear();
  js.position.clear();
  js.velocity.clear();
  js.effort.clear();
}

void resize(sensor_msgs::JointState& js, const size_t& size)
{
  js.name.resize(size);
  js.position.resize(size);
  js.velocity.resize(size);
  js.effort.resize(size);
}
}  // namespace tobas_ros
