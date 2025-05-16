#pragma once

#include "./joint_model.hpp"

namespace tobas
{
/* A planar joint */
class PlanarJointModel : public JointModel
{
public:
  /* different types of planar joints we support */
  enum MotionModel
  {
    HOLONOMIC,  // default
    DIFF_DRIVE
  };

  PlanarJointModel(const std::string& name, size_t joint_index, size_t first_variable_index);

  void getVariableDefaultPositions(double* values, const Bounds& other_bounds) const override;
  void getVariableRandomPositions(random_numbers::RandomNumberGenerator& rng, double* values, const Bounds& other_bounds)
    const override;
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

  double getMinTranslationalDistance() const
  {
    return min_translational_distance_;
  }

  void setMinTranslationalDistance(double min_translational_distance)
  {
    min_translational_distance_ = min_translational_distance;
  }

  MotionModel getMotionModel() const
  {
    return motion_model_;
  }

  void setMotionModel(MotionModel model)
  {
    motion_model_ = model;
  }

  /**
   * @brief Make the yaw component of a state's value vector be in the range [-Pi, Pi].
   * enforceBounds() also calls this function.
   * Return true if a change is actually made.
   */
  bool normalizeRotation(double* values) const;

private:
  double angular_distance_weight_;
  MotionModel motion_model_;
  // Only used for the differential drive motion model @see computeTurnDriveTurnGeometry
  double min_translational_distance_;
};

/**
 * @brief Compute the geometry to turn toward the target point, drive straight and then turn to target orientation
 *
 * @param[in] from                       A vector representing the initial position [x0, y0, theta0]
 * @param[in] to                         A vector representing the target position  [x1, y1, theta1]
 * @param[in] min_translational_distance If the translational distance between \p from and \p to is less than this value
 *                                       the motion will be pure rotation (meters)
 *
 * @param[out] dx           x1 - x0 (meters)
 * @param[out] dy           y1 - y0 (meters)
 * @param[out] initial_turn The initial turn in radians to face the target
 * @param[out] drive_angle  The orientation in radians that the robot will be driving straight at
 * @param[out] final_turn   The final turn in radians to the target orientation
 */
void computeTurnDriveTurnGeometry(
  const double* from,
  const double* to,
  const double min_translational_distance,
  double& dx,
  double& dy,
  double& initial_turn,
  double& drive_angle,
  double& final_turn);
}  // namespace tobas
