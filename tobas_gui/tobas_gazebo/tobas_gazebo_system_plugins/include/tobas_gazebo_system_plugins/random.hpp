// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <random>

#include <gz/math/Vector3.hh>

#include "./common/definitions.hpp"

namespace tobas
{
namespace gazebo
{
class NormalDistribution3d
{
public:
  using SharedPtr = std::shared_ptr<NormalDistribution3d>;

  explicit NormalDistribution3d(
    std::random_device& rnd_dev,
    const gz::math::Vector3d& mean,
    const gz::math::Vector3d& stddev);

  explicit NormalDistribution3d(std::random_device& rnd_dev, double mean, double stddev);

  gz::math::Vector3d get();

private:
  std::mt19937 rnd_gen_;
  NormalDistribution noise_[3];
  gz::math::Vector3d values_;
};

class UniformDistribution3d
{
public:
  using SharedPtr = std::shared_ptr<UniformDistribution3d>;

  explicit UniformDistribution3d(std::random_device& rnd_dev, const gz::math::Vector3d& lb, const gz::math::Vector3d& ub);

  explicit UniformDistribution3d(std::random_device& rnd_dev, double lb, double ub);

  gz::math::Vector3d get();

private:
  std::mt19937 rnd_gen_;
  UniformDistribution noise_[3];
  gz::math::Vector3d values_;
};

/* Generate a random point on the unit sphere. */
gz::math::Vector3d createUnitSpherePoint(std::random_device& rnd_dev);
}  // namespace gazebo
}  // namespace tobas
