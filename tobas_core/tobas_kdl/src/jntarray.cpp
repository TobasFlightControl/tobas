#include "../include/tobas_kdl/jntarray.hpp"

using namespace std;

namespace kdl
{
ostream& operator<<(ostream& os, const JntArray& arg)
{
  os << arg.data.transpose();
  return os;
}
}  // namespace kdl
