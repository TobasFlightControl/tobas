#pragma once

#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_drone_core/drone.hpp>

#include "./solver_i.hpp"

namespace tobas
{
/**
 * @brief モーメントに関する空力安定微係数の参照フレームを空力中心周りから重心周り変換する．
 */
class StabilityDerivativesCG : public SolverI
{
public:
  explicit StabilityDerivativesCG(const Drone& drone, const kdl::Tree& tree);

  void updateInternalDataStructures() override;

  int update(const kdl::JntArray& q);

  inline const double& cPitchAlpha() const;
  inline const double& cYawBeta() const;
  inline const double& cPitchDelta(size_t channel) const;
  inline const double& cYawDelta(size_t channel) const;

private:
  const Drone& drone_;
  const kdl::Tree& tree_;

  kdl::TreeInertiaSolver inertia_solver_;

  double c_pitch_alpha_cg_;
  double c_yaw_beta_cg_;
  std::map<size_t, double> c_pitch_delta_cg_;
  std::map<size_t, double> c_yaw_delta_cg_;
};

inline const double& StabilityDerivativesCG::cPitchAlpha() const
{
  return c_pitch_alpha_cg_;
}

inline const double& StabilityDerivativesCG::cYawBeta() const
{
  return c_yaw_beta_cg_;
}

inline const double& StabilityDerivativesCG::cPitchDelta(size_t channel) const
{
  return c_pitch_delta_cg_.at(channel);
}

inline const double& StabilityDerivativesCG::cYawDelta(size_t channel) const
{
  return c_yaw_delta_cg_.at(channel);
}
}  // namespace tobas
