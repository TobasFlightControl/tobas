#include "../include/tobas_rviz_plugin/fixed_joint_model.hpp"

namespace tobas
{
FixedJointModel::FixedJointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : JointModel(name, joint_index, first_variable_index)
{
  type_ = FIXED;
}

unsigned int FixedJointModel::getStateSpaceDimension() const
{
  return 0;
}

void FixedJointModel::getVariableDefaultPositions(double* /*values*/, const Bounds& /*bounds*/) const
{
}

void FixedJointModel::getVariableRandomPositions(
  random_numbers::RandomNumberGenerator& /*rng*/,
  double* /*values*/,
  const Bounds& /*bounds*/) const
{
}

void FixedJointModel::getVariableRandomPositionsNearBy(
  random_numbers::RandomNumberGenerator& /*rng*/,
  double* /*values*/,
  const Bounds& /*bounds*/,
  const double* /*near*/,
  const double /*distance*/) const
{
}

bool FixedJointModel::enforcePositionBounds(double* /*values*/, const Bounds& /*bounds*/) const
{
  return false;
}

bool FixedJointModel::satisfiesPositionBounds(const double* /*values*/, const Bounds& /*bounds*/, double /*margin*/)
  const
{
  return true;
}

double FixedJointModel::distance(const double* /*values1*/, const double* /*values2*/) const
{
  return 0.;
}

double FixedJointModel::getMaximumExtent(const Bounds& /*other_bounds*/) const
{
  return 0.;
}

void FixedJointModel::interpolate(const double* /*from*/, const double* /*to*/, const double /*t*/, double* /*state*/)
  const
{
}

void FixedJointModel::computeTransform(const double* /* joint_values */, Eigen::Isometry3d& transf) const
{
  transf.setIdentity();
}

void FixedJointModel::computeVariablePositions(const Eigen::Isometry3d& /* transform */, double* /* joint_values */)
  const
{
}
}  // namespace tobas
