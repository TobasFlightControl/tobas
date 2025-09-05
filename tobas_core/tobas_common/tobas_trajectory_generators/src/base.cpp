#include "tobas_trajectory_generators/base.hpp"

namespace traj
{
void BaseTrajectory::get(const double& t, double& p)
{
  get(t, p, dummy_, dummy_);
}

void BaseTrajectory::get(const double& t, double& p, double& v)
{
  get(t, p, v, dummy_);
}
}  // namespace traj
