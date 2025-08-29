#pragma once

#include <tobas_drone_tools/mixer_i.hpp>
#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>

namespace tobas
{
namespace y_axis_tilt_multicopter
{
/* memo: 3-39 */
class Mixer : public MixerI
{
  using super = MixerI;

  static constexpr double kMinVerticalForcePerMass = 5.;  // [m/s^2]

public:
  explicit Mixer(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;

  bool solve(
    const kdl::JntArray& cur_q,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_dgyro_B,
    const double& ux,
    const double& uz,
    const kdl::Vector& ext_torque_B = kdl::Vector::Zero());

  double getThrust(size_t idx) const;
  double getTiltAngle(size_t idx) const;

private:
  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  Eigen::Matrix5Xd E_;
  Eigen::Vector5d f_;
  Eigen::VectorXd x_;

  std::map<std::string, kdl::Vector> thrust_points_;  // 祖父母リンクから見た推力の作用点
};
}  // namespace y_axis_tilt_multicopter
}  // namespace tobas
