#pragma once

#include <Eigen/Core>

#include <tobas_kdl/frames.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_kdl/treefksolverpos.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>

#include <tobas_tools/drone.hpp>
#include <tobas_tools/rotor_axis_extractor.hpp>

namespace tobas_mr_pid
{
/**
 * @brief マルチコプターの目標角加速度と合計推力から各プロペラの推力を計算する．
 * ダイナミクスの時間発展を考えない代わりにコリオリ力を考慮できるのが利点．
 */
class Mixer
{
public:
  explicit Mixer(const tobas::Drone& drone);

  void updateInternalDataStructures();

  Eigen::VectorXd solve(
    const KDL::Vector& cur_gyro_B,
    const KDL::Vector& tar_dgyro_B,
    const KDL::JntArray& q,
    const double& tar_thrust_sum);

private:
  const tobas::Drone& drone_;

  KDL::TreeFkSolverPos fk_solver_;
  KDL::TreeJntToInertiaSolver inertia_solver_;
  tobas::RotorAxisExtractor z_rotors_;

  Eigen::Matrix4Xd A_;
  Eigen::Vector4d b_;

  KDL::Vector P_base_cog_;
  Eigen::Vector3d P_cog_rotor_eigen_;
  KDL::Frame T_base_rotor_;
  KDL::RotationalInertia I_cog_kdl_;  // CoG周りの回転慣性テンソル
  Eigen::Matrix3d I_cog_eigen_;       // CoG周りの回転慣性テンソル
  Eigen::Vector3d cur_gyro_eigen_;
  Eigen::Vector3d tar_dgyro_eigen_;
};
}  // namespace tobas_mr_pid
