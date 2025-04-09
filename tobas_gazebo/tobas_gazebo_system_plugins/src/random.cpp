#include "../include/tobas_gazebo_system_plugins/random.hpp"

using namespace std;
using namespace gz::math;

namespace gazebo
{
NormalDistribution3d::NormalDistribution3d(random_device& rnd_dev, const Vector3d& mean, const Vector3d& stddev)
  : rnd_gen_(rnd_dev())
{
  for (size_t i = 0; i < 3; ++i)
    noise_[i] = NormalDistribution(mean[i], stddev[i]);
}

NormalDistribution3d::NormalDistribution3d(random_device& rnd_dev, double mean, double stddev)
  : NormalDistribution3d(rnd_dev, mean * Vector3d::One, stddev * Vector3d::One)
{
}

Vector3d NormalDistribution3d::get()
{
  for (size_t i = 0; i < 3; ++i)
    values_[i] = noise_[i](rnd_gen_);
  return values_;
}

UniformDistribution3d::UniformDistribution3d(random_device& rnd_dev, const Vector3d& lb, const Vector3d& ub)
  : rnd_gen_(rnd_dev())
{
  for (size_t i = 0; i < 3; ++i)
    noise_[i] = UniformDistribution(lb[i], ub[i]);
}

UniformDistribution3d::UniformDistribution3d(random_device& rnd_dev, double lb, double ub)
  : UniformDistribution3d(rnd_dev, lb * Vector3d::One, ub * Vector3d::One)
{
}

Vector3d UniformDistribution3d::get()
{
  for (size_t i = 0; i < 3; ++i)
    values_[i] = noise_[i](rnd_gen_);
  return values_;
}

Vector3d createUnitSpherePoint(random_device& rnd_dev)
{
  UniformDistribution angle_dist(-M_PI, M_PI);

  const auto phi = angle_dist(rnd_dev);
  const auto theta = angle_dist(rnd_dev);

  return Vector3d(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));
}
}  // namespace gazebo
