#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include "./jntspaceinertiamatrix.hpp"
#include "./chainjnttojacsolver.hpp"
#include "./chainfksolverpos_recursive.hpp"
#include "./chainfksolvervel_recursive.hpp"
#include "./chainiksolvervel_pinv.hpp"
#include "./chainiksolveracc_rac.hpp"
#include "./chainidsolver_rne.hpp"
#include "./chaindynparam.hpp"
#include "./chainjnttoinertiasolver.hpp"

namespace KDL
{
class ChainKDLModel : public SolverI
{
public:
  ChainKDLModel(const Chain& chain);

  void updateInternalDataStructures() override;

  /* ヤコビアンを計算する */
  void segmentJacobian(const JntArray& q, const std::string& name, Jacobian& jac);

  /* 指定したフレームの位置を求める */
  void fkPos(const JntArray& q, const std::string& name, Frame& frame);

  /* 指定したフレームの位置と速度を求める */
  void fkVel(const JntArrayVel& q, const std::string& name, FrameVel& frame);

  /* 並進速度に関する逆運動学 */
  void ikVel(const Vector& v, const JntArray& q, JntArray& qd);

  /* ツイストに関する逆運動学 */
  void ikVel(const Twist& v, const JntArray& q, JntArray& qd);

  /* 分解加速度制御 */
  void ikAcc(const Vector& a, const JntArray& q, const JntArray& qd, JntArray& qdd);

  /* 分解加速度制御 */
  void ikAcc(const Twist& a, const JntArray& q, const JntArray& qd, JntArray& qdd);

  /* 分解加速度制御の計算過程で登場するJdqdのみを求める */
  void calcJdqd(const JntArray& q, const JntArray& qd, Twist& Jdqd);

  /* 指定したリンクのリンク原点から見た慣性データを返す */
  const RigidBodyInertia& segmentInertia(const std::string& name);

  /* {root}から見た{root}を除くchain全体の慣性データを返す */
  RigidBodyInertia chainInertia(const KDL::JntArray& q);

  /* 各リンクへの重力以外の外力を考慮した場合のRNE */
  void inverseDynamics(
    const JntArray& q,
    const JntArray& qd,
    const JntArray& qdd,
    const Wrenches& forces,
    const Vector& grav,
    JntArray& torques);

  /* EEへの反力のみを考慮したRNE */
  void inverseDynamics(
    const JntArray& q,
    const JntArray& qd,
    const JntArray& qdd,
    const Wrench& f_ee,
    const Vector& grav,
    JntArray& torques);

  /* 関節空間における慣性行列M(q)を計算する */
  void jntSpaceInertiaMatrix(const JntArray& q, JntSpaceInertiaMatrix& inertia);

  /* コリオリ力によって各関節にかかるトルクを計算する */
  void coriolisEffort(const JntArray& q, const JntArray& qd, JntArray& torque);

  /* 重力によって各関節にかかるトルクを計算する */
  void gravityEffort(const JntArray& q, const Vector& grav, JntArray& torque);

  const std::vector<std::string>& jntNames() const;

private:
  const Chain& chain_;
  int nj_;
  int ns_;
  std::vector<std::string> jnt_names_;
  std::unordered_map<std::string, int> seg2idx_;
  ChainJntToJacSolver jnt2jac_;
  ChainFkSolverPos_recursive fk_pos_;
  ChainFkSolverVel_recursive fk_vel_;
  ChainIkSolverVel_pinv ik_vel_;
  ChainIkSolverAcc_RAC ik_acc_;
  ChainIdSolver_RNE ext_rne_;
  ChainDynParam ext_dynparam_;
  ChainJntToInertiaSolver inertia_solver_;

  void parseJntNames();

  int getSegIdx(const std::string& name);
};
}  // namespace KDL
