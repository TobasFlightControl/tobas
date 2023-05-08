#pragma once

#include <Eigen/Core>

#include <dh_linear_control/state_spaces.hpp>
#include <dh_kdl/treefksolverpos.hpp>
#include <dh_kdl/treejnttoinertiasolver.hpp>

#include <tobas_tools/drone.hpp>

#define STATE_SIZE 6  // 姿勢制御器の状態の次元

// 状態ベクトルにおける各変数のインデックス
#define ROLL 0
#define PITCH 1
#define YAW 2
#define ANGVEL_X 3
#define ANGVEL_Y 4
#define ANGVEL_Z 5

namespace tobas_multirotor_controller
{
/**
 * @brief クアッドロータの連続時間状態方程式．
 */
class MultiRotorDynamics : public ctrl::LinearDynamics
{
public:
  /**
   * @brief Construct a new MultiRotorDynamics object
   *
   * @param tree 全身のTree
   */
  explicit MultiRotorDynamics(const Drone& drone);

  /**
   * @brief 状態方程式を更新する．
   *
   * @param roll {world}に対する{base}のロール角
   * @param pitch {world}に対する{base}のピッチ角
   * @param q アームの関節角
   */
  void update(const double& roll, const double& pitch, const KDL::JntArray& q);

private:
  const Drone& drone_;
  const std::vector<uint32_t> ver_prop_idxes_;
  const uint32_t u_dim_;  // 制御入力の次元

  // KDL tools
  KDL::ExtTreeFkSolverPos fk_solver_;
  KDL::TreeJntToInertiaSolver inertia_solver_;

  KDL::Rotation rpyvel_angvel_kdl_;
  Eigen::Matrix3d rpyvel_angvel_eigen_;
  bool invertible_;
  KDL::Vector P_base_cog_;
  KDL::Vector P_cog_rotor_kdl_;
  Eigen::Vector3d P_cog_rotor_eigen_;
  KDL::Frame T_base_rotor_;
  KDL::RotationalInertia I_cog_kdl_;  // CoG周りの回転慣性テンソル
  Eigen::Matrix3d I_cog_eigen_;       // CoG周りの回転慣性テンソル
  Eigen::Matrix3d I_cog_inv_;

  void updateA(const double& roll, const double& pitch);
  void updateB(const KDL::JntArray& q);
};
}  // namespace tobas_multirotor_controller
