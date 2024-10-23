#include <iostream>

#include "../include/tobas_kdl/euler.hpp"

using namespace std;

namespace kdl
{
bool Euler::isFinite() const
{
  return isfinite(roll) && isfinite(pitch) && isfinite(yaw);
}

inline ostream& operator<<(ostream& os, const Euler& arg)
{
  os << "roll: " << arg.roll << ", pitch: " << arg.pitch << ", yaw: " << arg.yaw;
  return os;
}
}  // namespace kdl
