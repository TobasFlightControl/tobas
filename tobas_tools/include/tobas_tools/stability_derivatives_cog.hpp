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

  inline double cPitchAlpha() const;
  inline double cYawBeta() const;
  inline double cPitchDelta(const size_t& cs_idx) const;
  inline double cYawDelta(const size_t& cs_idx) const;

private:
  const Drone& drone_;

  KDL::TreeJntToInertiaSolver inertia_solver_;

  double c_pitch_alpha_cg_;
  double c_yaw_beta_cg_;
  std::vector<double> c_pitch_delta_cg_;
  std::vector<double> c_yaw_delta_cg_;
};

inline double StabilityDerivativesCG::cPitchAlpha() const
{
  return c_pitch_alpha_cg_;
}

inline double StabilityDerivativesCG::cYawBeta() const
{
  return c_yaw_beta_cg_;
}

inline double StabilityDerivativesCG::cPitchDelta(const size_t& cs_idx) const
{
  return c_pitch_delta_cg_[cs_idx];
}

inline double StabilityDerivativesCG::cYawDelta(const size_t& cs_idx) const
{
  return c_yaw_delta_cg_[cs_idx];
}
}  // namespace tobas
