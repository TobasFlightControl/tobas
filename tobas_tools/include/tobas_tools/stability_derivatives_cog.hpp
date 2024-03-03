#pragma once

#include <tobas_kdl/treejnttoinertiasolver.hpp>

#include "./solveri.hpp"
#include "./drone.hpp"

namespace tobas
{
/**
 * @brief モーメントに関する空力安定微係数の参照フレームを空力中心周りから重心周り変換する．
 */
class StabilityDerivativesCG : public SolverI
{
public:
  explicit StabilityDerivativesCG(const Drone& drone);

  void updateInternalDataStructures() override;

  int update(const KDL::JntArray& q);

  inline const double& cPitchAlpha() const;
  inline const double& cYawBeta() const;
  inline const double& cPitchDelta(const size_t& cs_idx) const;
  inline const double& cYawDelta(const size_t& cs_idx) const;

private:
  const Drone& drone_;

  KDL::TreeJntToInertiaSolver inertia_solver_;

  double c_pitch_alpha_cg_;
  double c_yaw_beta_cg_;
  std::vector<double> c_pitch_delta_cg_;
  std::vector<double> c_yaw_delta_cg_;
};

inline const double& StabilityDerivativesCG::cPitchAlpha() const
{
  return c_pitch_alpha_cg_;
}

inline const double& StabilityDerivativesCG::cYawBeta() const
{
  return c_yaw_beta_cg_;
}

inline const double& StabilityDerivativesCG::cPitchDelta(const size_t& cs_idx) const
{
  return c_pitch_delta_cg_[cs_idx];
}

inline const double& StabilityDerivativesCG::cYawDelta(const size_t& cs_idx) const
{
  return c_yaw_delta_cg_[cs_idx];
}
}  // namespace tobas
