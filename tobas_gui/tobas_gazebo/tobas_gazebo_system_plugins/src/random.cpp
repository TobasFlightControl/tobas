// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/random.hpp"

namespace tobas
{
namespace gazebo
{
NormalDistribution3d::NormalDistribution3d(
  std::random_device& rnd_dev,
  const gz::math::Vector3d& mean,
  const gz::math::Vector3d& stddev)
  : rnd_gen_(rnd_dev())
{
  for (size_t i = 0; i < 3; ++i) {
    noise_[i] = NormalDistribution(mean[i], stddev[i]);
  }
}

NormalDistribution3d::NormalDistribution3d(std::random_device& rnd_dev, double mean, double stddev)
  : NormalDistribution3d(rnd_dev, mean * gz::math::Vector3d::One, stddev * gz::math::Vector3d::One)
{
}

gz::math::Vector3d NormalDistribution3d::get()
{
  for (size_t i = 0; i < 3; ++i) {
    values_[i] = noise_[i](rnd_gen_);
  }
  return values_;
}

UniformDistribution3d::UniformDistribution3d(
  std::random_device& rnd_dev,
  const gz::math::Vector3d& lb,
  const gz::math::Vector3d& ub)
  : rnd_gen_(rnd_dev())
{
  for (size_t i = 0; i < 3; ++i) {
    noise_[i] = UniformDistribution(lb[i], ub[i]);
  }
}

UniformDistribution3d::UniformDistribution3d(std::random_device& rnd_dev, double lb, double ub)
  : UniformDistribution3d(rnd_dev, lb * gz::math::Vector3d::One, ub * gz::math::Vector3d::One)
{
}

gz::math::Vector3d UniformDistribution3d::get()
{
  for (size_t i = 0; i < 3; ++i) {
    values_[i] = noise_[i](rnd_gen_);
  }
  return values_;
}

gz::math::Vector3d createUnitSpherePoint(std::random_device& rnd_dev)
{
  UniformDistribution angle_dist(-M_PI, M_PI);

  const auto phi = angle_dist(rnd_dev);
  const auto theta = angle_dist(rnd_dev);

  return gz::math::Vector3d(std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta), std::cos(phi));
}
}  // namespace gazebo
}  // namespace tobas
