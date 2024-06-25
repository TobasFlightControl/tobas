#pragma once

#include <random>
#include <gazebo/gazebo.hh>

#include "./common.hpp"

namespace gazebo
{
class NormalDistribution3d
{
public:
  explicit NormalDistribution3d(
    std::random_device& rnd_dev,
    const ignition::math::Vector3d& mean,
    const ignition::math::Vector3d& stddev);

  ignition::math::Vector3d get();

private:
  std::mt19937 rnd_gen_;
  NormalDistribution noise_[3];
  ignition::math::Vector3d values_;
};

class UniformDistribution3d
{
public:
  explicit UniformDistribution3d(
    std::random_device& rnd_dev,
    const ignition::math::Vector3d& lb,
    const ignition::math::Vector3d& ub);

  ignition::math::Vector3d get();

private:
  std::mt19937 rnd_gen_;
  UniformDistribution noise_[3];
  ignition::math::Vector3d values_;
};

using NormalDistribution3dPtr = std::shared_ptr<NormalDistribution3d>;
using UniformDistribution3dPtr = std::shared_ptr<UniformDistribution3d>;
}  // namespace gazebo
