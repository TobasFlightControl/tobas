#pragma once

#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>

#include "./mixer.hpp"
#include "./rotor_axis_extractor.hpp"

namespace tobas
{
/**
 * @brief 疑似逆行列を用いたマルチコプターの推力ミキシング
 */
class MultiRotorMixer_pinv : public Mixer
{
  using super = Mixer;

  static constexpr double kThrustClampMargin = 1e-3;  // [N]

public:
  explicit MultiRotorMixer_pinv(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;

  bool solve(
    const kdl::JntArray& cur_q,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_dgyro_B,
    const double& tar_thrusts_sum,
    const kdl::Vector& ext_torque_B = kdl::Vector::Zero());

  const Eigen::VectorXd& getThrusts() const;

private:
  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;
  RotorAxisExtractor z_rotors_;

  Eigen::Matrix4Xd E_;
  Eigen::Vector4d f_;
  Eigen::VectorXd x_;
};
}  // namespace tobas
