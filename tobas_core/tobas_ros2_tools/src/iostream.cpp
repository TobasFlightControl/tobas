#include "../include/tobas_ros2_tools/iostream.hpp"

using namespace std;

ostream& operator<<(ostream& os, const geometry_msgs::msg::Quaternion& q)
{
  os << "x: " << q.x << ", y: " << q.y << ", z: " << q.z << ", w: " << q.w;
  return os;
}
