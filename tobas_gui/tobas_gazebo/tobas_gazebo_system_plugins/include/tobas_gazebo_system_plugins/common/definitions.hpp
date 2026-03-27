#pragma once

#include <ext/random>
#include <random>

namespace tobas
{
namespace gazebo
{
using NormalDistribution = std::normal_distribution<double>;
using UniformDistribution = std::uniform_real_distribution<double>;
using RiceDistribution = __gnu_cxx::rice_distribution<double>;
}  // namespace gazebo
}  // namespace tobas
