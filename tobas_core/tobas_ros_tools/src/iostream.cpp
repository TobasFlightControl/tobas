#include "../include/tobas_ros_tools/iostream.hpp"

using namespace std;

ostream& operator<<(ostream& os, const geometry_msgs::Quaternion& q)
{
  os << "x: " << q.x << ", y: " << q.y << ", z: " << q.z << ", w: " << q.w;
  return os;
}
