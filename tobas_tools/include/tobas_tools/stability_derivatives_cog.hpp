#pragma once

#include <dh_kdl/treejnttoinertiasolver.hpp>

#include "./drone.hpp"

namespace tobas
{
/**
 * @brief モーメントに関する空力安定微係数の参照フレームを空力中心周りから重心周り変換する．
 */
class StabilityDerivativesCG
{
public:
  explicit StabilityDerivativesCG(const Drone& drone);

  void updateInternalDataStructures();

  void update(const KDL::JntArray& q);

  double cPitchAlpha() const;
  double cYawBeta() const;
  double cPitchDelta(uint32_t cs_idx) const;
  double cYawDelta(uint32_t cs_idx) const;

private:
  const Drone& drone_;

  KDL::TreeJntToInertiaSolver inertia_solver_;

  KDL::Vector cog_;
  KDL::RotationalInertia I_;

  double c_pitch_alpha_cg_;
  double c_yaw_beta_cg_;
  std::vector<double> c_pitch_delta_cg_;
  std::vector<double> c_yaw_delta_cg_;
};
}  // namespace tobas
