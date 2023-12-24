#include "../include/dh_eigen_tools/iostream.hpp"

using namespace std;
using namespace Eigen;

ostream& operator<<(ostream& os, const Quaterniond& arg)
{
  os << "x: " << arg.x() << ", y: " << arg.y() << ", z: " << arg.z() << ", w: " << arg.w();
  return os;
}
