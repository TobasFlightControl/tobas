// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/random.hpp"

namespace tobas::gazebo
{
NormalDistribution3d::NormalDistribution3d(
  std::random_device& _rnd_dev,
  const gz::math::Vector3d& _mean,
  const gz::math::Vector3d& _stddev)
  : rnd_gen_(_rnd_dev())
{
  for (size_t i = 0; i < 3; ++i) {
    noise_[i] = NormalDistribution(_mean[i], _stddev[i]);
  }
}

NormalDistribution3d::NormalDistribution3d(std::random_device& _rnd_dev, double _mean, double _stddev)
  : NormalDistribution3d(_rnd_dev, _mean * gz::math::Vector3d::One, _stddev * gz::math::Vector3d::One)
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
  std::random_device& _rnd_dev,
  const gz::math::Vector3d& _lb,
  const gz::math::Vector3d& _ub)
  : rnd_gen_(_rnd_dev())
{
  for (size_t i = 0; i < 3; ++i) {
    noise_[i] = UniformDistribution(_lb[i], _ub[i]);
  }
}

UniformDistribution3d::UniformDistribution3d(std::random_device& _rnd_dev, double _lb, double _ub)
  : UniformDistribution3d(_rnd_dev, _lb * gz::math::Vector3d::One, _ub * gz::math::Vector3d::One)
{
}

gz::math::Vector3d UniformDistribution3d::get()
{
  for (size_t i = 0; i < 3; ++i) {
    values_[i] = noise_[i](rnd_gen_);
  }
  return values_;
}

gz::math::Vector3d createUnitSpherePoint(std::random_device& _rnd_dev)
{
  UniformDistribution angle_dist(-M_PI, M_PI);

  const auto phi = angle_dist(_rnd_dev);
  const auto theta = angle_dist(_rnd_dev);

  return gz::math::Vector3d(std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta), std::cos(phi));
}
}  // namespace tobas::gazebo
