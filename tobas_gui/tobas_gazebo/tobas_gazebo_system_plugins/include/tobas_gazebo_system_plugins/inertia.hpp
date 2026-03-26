#pragma once

#include <tuple>

namespace tobas
{
namespace gazebo
{
std::tuple<double, double, double> boxInertia(double sx, double sy, double sz, double mass);
}  // namespace gazebo
}  // namespace tobas
