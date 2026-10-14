// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <random>

#include <gz/math/Vector3.hh>

#include "./common/definitions.hpp"

namespace tobas::gazebo
{
class NormalDistribution3d
{
public:
  explicit NormalDistribution3d(
    std::random_device& _rnd_dev,
    const gz::math::Vector3d& _mean,
    const gz::math::Vector3d& _stddev);
  explicit NormalDistribution3d(std::random_device& _rnd_dev, double _mean, double _stddev);

  gz::math::Vector3d get();

private:
  std::mt19937 rnd_gen_;
  NormalDistribution noise_[3];
  gz::math::Vector3d values_;
};

class UniformDistribution3d
{
public:
  explicit UniformDistribution3d(
    std::random_device& _rnd_dev,
    const gz::math::Vector3d& _lb,
    const gz::math::Vector3d& _ub);
  explicit UniformDistribution3d(std::random_device& _rnd_dev, double _lb, double _ub);

  gz::math::Vector3d get();

private:
  std::mt19937 rnd_gen_;
  UniformDistribution noise_[3];
  gz::math::Vector3d values_;
};

/** Generate a random point on the unit sphere. */
gz::math::Vector3d createUnitSpherePoint(std::random_device& _rnd_dev);
}  // namespace tobas::gazebo
