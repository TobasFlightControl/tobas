#pragma once

#include <tuple>

namespace gazebo
{
std::tuple<double, double, double> boxInertia(double sx, double sy, double sz, double mass);
}  // namespace gazebo
