#pragma once

#include "./joint_model.hpp"

namespace tobas
{
/* A floating joint */
class FloatingJointModel : public JointModel
{
public:
  FloatingJointModel(const std::string& name, size_t joint_index, size_t first_variable_index);

  void getVariableDefaultPositions(double* values, const Bounds& other_bounds) const override;
  void getVariableRandomPositions(
    random_numbers::RandomNumberGenerator& rng,
    double* values,
    const Bounds& other_bounds) const override;
  void getVariableRandomPositionsNearBy(
    random_numbers::RandomNumberGenerator& rng,
    double* values,
    const Bounds& other_bounds,
    const double* near,
    const double distance) const override;
  bool enforcePositionBounds(double* values, const Bounds& other_bounds) const override;
  bool satisfiesPositionBounds(const double* values, const Bounds& other_bounds, double margin) const override;

  void interpolate(const double* from, const double* to, const double t, double* state) const override;
  unsigned int getStateSpaceDimension() const override;
  double getMaximumExtent(const Bounds& other_bounds) const override;
  double distance(const double* values1, const double* values2) const override;

  void computeTransform(const double* joint_values, Eigen::Isometry3d& transf) const override;
  void computeVariablePositions(const Eigen::Isometry3d& transf, double* joint_values) const override;

  double getAngularDistanceWeight() const
  {
    return angular_distance_weight_;
  }

  void setAngularDistanceWeight(double weight)
  {
    angular_distance_weight_ = weight;
  }

  /* Normalize the quaternion (warn if norm is 0, and set to identity); Return true if any change was made. */
  bool normalizeRotation(double* values) const;

  /* Get the distance between the rotation components of two states. */
  double distanceRotation(const double* values1, const double* values2) const;

  /* Get the distance between the translation components of two states. */
  double distanceTranslation(const double* values1, const double* values2) const;

private:
  double angular_distance_weight_;
};
}  // namespace tobas
