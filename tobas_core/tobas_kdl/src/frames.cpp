#include <algorithm>
#include <iostream>

#include "../include/tobas_kdl/frames.hpp"
#include "../include/tobas_kdl/utilities/utility.hpp"

using namespace std;
using namespace Eigen;

namespace kdl
{
ostream& operator<<(ostream& os, const Vector& v)
{
  os << "[" << v.x() << "," << v.y() << "," << v.z() << "]";
  return os;
}

ostream& operator<<(ostream& os, const Rotation& R)
{
  double r, p, y;
  R.getRPY(r, p, y);
  os << "[RPY]" << endl;
  os << "[";
  os << r << ",";
  os << p << ",";
  os << y << "]";
  return os;
}

ostream& operator<<(ostream& os, const Frame& T)
{
  os << "[" << T.M << endl << T.p << "]";
  return os;
}

ostream& operator<<(ostream& os, const Twist& v)
{
  os << "[" << v.vel.x() << "," << v.vel.y() << "," << v.vel.z() << "," << v.rot.x() << "," << v.rot.y() << ","
     << v.rot.z() << "]";
  return os;
}

ostream& operator<<(ostream& os, const Accel& v)
{
  os << "[" << v.linear.x() << "," << v.linear.y() << "," << v.linear.z() << "," << v.angular.x() << ","
     << v.angular.y() << "," << v.angular.z() << "]";
  return os;
}

ostream& operator<<(ostream& os, const Wrench& v)
{
  os << "[" << v.force.x() << "," << v.force.y() << "," << v.force.z() << "," << v.torque.x() << "," << v.torque.y()
     << "," << v.torque.z() << "]";
  return os;
}
}  // namespace kdl
