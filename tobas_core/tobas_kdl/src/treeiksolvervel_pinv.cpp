#include <iostream>

#include <tobas_std_tools/unordered_set.hpp>
#include <tobas_eigen_tools/linalg.hpp>
#include <tobas_quadprog/utils.hpp>

#include "../include/tobas_kdl/treeiksolvervel_pinv.hpp"

#define INF numeric_limits<double>::max()

using namespace std;
using namespace Eigen;

namespace KDL
{
TreeIkSolverVel_pinv::TreeIkSolverVel_pinv(const Tree& tree)
  : super(tree), jnt2jac_(tree_), jntparser_(tree_)
{
  updateInternalDataStructures();
}

void TreeIkSolverVel_pinv::updateInternalDataStructures()
{
  super::updateInternalDataStructures();
  jnt2jac_.updateInternalDataStructures();
  jntparser_.updateInternalDataStructures();

  qp_solver_.x_scale = VectorXd::Ones(nj_);
  qp_solver_.problem.G.conservativeResize(0, nj_);
  qp_solver_.problem.h.conservativeResize(0);
}

int TreeIkSolverVel_pinv::CartToJnt(const JntArray& q_in, const TwistMap& v_in)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q_in.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  const auto num_points = v_in.size();
  const auto eq_dim = 6 * num_points;

  // Create big jacobian and velocity
  MatrixXd J(eq_dim, nj_);
  VectorXd t(eq_dim);
  size_t i = 0;
  for (const auto& [seg_name, twist] : v_in)
  {
    // Update big jacobian
    if (jnt2jac_.JntToJac(q_in, seg_name) < 0)
      return copyError(jnt2jac_);
    J.block(6 * i, 0, 6, nj_) = jnt2jac_.getJacobian().data;

    // Update big velocity
    t.segment(6 * i, 3) = twist.vel.data;
    t.segment(6 * i + 3, 3) = twist.rot.data;

    ++i;
  }

  // 評価関数
  const VectorXd Wt = eigen_tools::tile(Wt_, num_points, 0);
  const VectorXd Wj = VectorXd::Constant(nj_, Wj_);
  const MatrixXd JT_Wt = J.transpose() * Wt.asDiagonal();
  qp_solver_.problem.P = JT_Wt * J;
  qp_solver_.problem.P.diagonal() += Wj;
  qp_solver_.problem.q = -JT_Wt * t;

  // 不等式制約
  auto qd_min = -jntparser_.maxVelocities();
  auto qd_max = +jntparser_.maxVelocities();
  for (size_t j = 0; j < nj_; ++j)
  {
    // 既に関節角制限をオーバーしている場合は，それ以上違反量を大きくしないように制限
    if (q_in(j) < jntparser_.lowerLimit(j))
      qd_min(j) = 0.;
    else if (q_in(j) > jntparser_.upperLimit(j))
      qd_max(j) = 0.;
  }
  quadprog::matIneqFromRange(qd_min.data, qd_max.data, qp_solver_.problem.A, qp_solver_.problem.b);

  // QPを解く
  qd_out_.data = qp_solver_.solve();

  return setDefaultError(E_NOERROR);
}

bool TreeIkSolverVel_pinv::setWeightTS(const Vector6d& Wt)
{
  if ((Wt.array() < 0).any())
    return false;

  Wt_ = Wt;
  return true;
}

const Vector6d& TreeIkSolverVel_pinv::getWeightTS() const
{
  return Wt_;
}

bool TreeIkSolverVel_pinv::setWeightJS(const double& Wj)
{
  // 数値エラーを防ぐため正則化項は必ず入れる
  if (Wj <= 0)
    return false;

  Wj_ = Wj;
  return true;
}

const double& TreeIkSolverVel_pinv::getWeightJS() const
{
  return Wj_;
}
}  // namespace KDL
