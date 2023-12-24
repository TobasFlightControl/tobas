#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include "./utilities/constants.hpp"
#include "./treejntparser.hpp"
#include "./treejnttojacsolver.hpp"
#include "./treefksolverpos.hpp"
#include "./treefksolvervel.hpp"
#include "./treedynparam.hpp"
#include "./treejnttoinertiasolver.hpp"
#include "./treeidsolver_recursive_newton_euler.hpp"

namespace KDL
{
class TreeKDLModel : public SolverI
{
public:
  explicit TreeKDLModel(const Tree& tree, const Vector& grav = Vector(0., 0., -kDefaultGravity));

  void updateInternalDataStructures() override;

  /* Treeの全駆動関節数を返す． */
  size_t getNrOfJoints();

  /* Treeの全リンク数を返す． */
  size_t getNrOfSegments();

  /* ヤコビアンを計算する． */
  void segmentJacobian(const JntArray& q, const std::string& name, Jacobian& jac);

  /* 指定したフレームのルートリンクに対する位置を求める． */
  Frame fkPos(const JntArray& q, const std::string& name);

  /* 指定したフレームのルートリンクに対する位置と速度を求める． */
  FrameVel fkVel(const JntArray& q, const JntArray& qd, const std::string& name);

  /* Recursive Newton Euler */
  void inverseDynamics(
    const JntArray& q,
    const JntArray& qd,
    const JntArray& qdd,
    const WrenchMap& f_ext,
    JntArray& torques);

  /* Recursive Newton Euler */
  void
  inverseDynamics(const JntArray& q, const JntArray& qd, const JntArray& qdd, JntArray& torques);

  /* 関節空間における慣性行列M(q)を計算する． */
  void jntSpaceInertiaMatrix(const JntArray& q, const JntArray& qd, JntSpaceInertiaMatrix& inertia);

  /* 関節空間における慣性行列M(q)を計算する． */
  void jntSpaceInertiaMatrix(const JntArray& q, JntSpaceInertiaMatrix& inertia);

  /* コリオリ力によって各関節にかかるトルクを計算する． */
  void coriolisEffort(const JntArray& q, const JntArray& qd, JntArray& torque);

  /* 重力によって各関節にかかるトルクを計算する． */
  void gravityEffort(const JntArray& q, JntArray& torque);

  /* Tree全体の慣性データをRigidBodyInertiaで得る． */
  RigidBodyInertia treeInertia(const JntArray& q_in);

  /* Tree全体の質量を計算する． */
  double treeMass();

  /* KDLの順に並んだ駆動関節名のリストを返す． */
  const std::vector<std::string>& jointNames() const;

  /* 駆動関節名からインデックスへのマップ． */
  const std::unordered_map<std::string, size_t>& jointIndexMap() const;

  /* index -> joint name */
  const std::string& jointName(const int& idx) const;

  /**
   * @brief 関節名に対応するQNrを返す．
   *
   * @param name 関節名
   *
   * @throw 関節が存在しない場合はstd::out_of_range
   */
  const size_t& jointIndex(const std::string& name) const;

private:
  const Tree& tree_;

  WrenchMap wrenchmap_null_;

  TreeJointParser jnt_parser_;
  TreeJntToJacSolver jnt2jac_;
  TreeFkSolverPos fk_pos_;
  TreeFkSolverVel fk_vel_;
  TreeIdSolver_RNE rne_;
  TreeDynParam dynparam_;
  TreeJntToInertiaSolver inertia_solver_;
};
}  // namespace KDL
