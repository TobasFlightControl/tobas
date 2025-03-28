#pragma once

#include "./joint_model.hpp"

namespace tobas
{
/** \brief A revolute joint */
class RevoluteJointModel : public JointModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RevoluteJointModel(const std::string& name, size_t joint_index, size_t first_variable_index);
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
  bool harmonizePosition(double* values, const Bounds& other_bounds) const override;

  void interpolate(const double* from, const double* to, const double t, double* state) const override;
  unsigned int getStateSpaceDimension() const override;
  double getMaximumExtent(const Bounds& other_bounds) const override;
  double distance(const double* values1, const double* values2) const override;

  void computeTransform(const double* joint_values, Eigen::Isometry3d& transf) const override;
  void computeVariablePositions(const Eigen::Isometry3d& transf, double* joint_values) const override;

  void setContinuous(bool flag);

  /** \brief Check if this joint wraps around */
  bool isContinuous() const
  {
    return continuous_;
  }

  /** \brief Get the axis of rotation */
  const Eigen::Vector3d& getAxis() const
  {
    return axis_;
  }

  /** \brief Set the axis of rotation */
  void setAxis(const Eigen::Vector3d& axis);

protected:
  /** \brief The axis of the joint */
  Eigen::Vector3d axis_;

  /** \brief Flag indicating whether this joint wraps around */
  bool continuous_;

private:
  double x2_, y2_, z2_, xy_, xz_, yz_;
};
}  // namespace tobas
