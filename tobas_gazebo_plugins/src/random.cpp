#include "../include/tobas_gazebo_plugins/random.hpp"

using namespace ignition::math;

namespace gazebo
{
NormalDistribution3d::NormalDistribution3d(
  std::random_device& rnd_dev,
  const Vector3d& mean,
  const Vector3d& stddev)
  : rnd_gen_(rnd_dev())
{
  for (int i = 0; i < 3; ++i)
  {
    noise_[i] = NormalDistribution(mean[i], stddev[i]);
  }
}

Vector3d NormalDistribution3d::get()
{
  for (int i = 0; i < 3; ++i)
  {
    values_[i] = noise_[i](rnd_gen_);
  }
  return values_;
}

UniformDistribution3d::UniformDistribution3d(
  std::random_device& rnd_dev,
  const Vector3d& lb,
  const Vector3d& ub)
  : rnd_gen_(rnd_dev())
{
  for (int i = 0; i < 3; ++i)
  {
    noise_[i] = UniformDistribution(lb[i], ub[i]);
  }
}

Vector3d UniformDistribution3d::get()
{
  for (int i = 0; i < 3; ++i)
  {
    values_[i] = noise_[i](rnd_gen_);
  }
  return values_;
}
}  // namespace gazebo
