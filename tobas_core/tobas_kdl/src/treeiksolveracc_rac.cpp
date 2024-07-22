#include <tobas_eigen_tools/linalg.hpp>
#include <tobas_quadprog/utils.hpp>

#include "../include/tobas_kdl/treeiksolveracc_rac.hpp"

using namespace std;
using namespace Eigen;

namespace kdl
{
TreeIkSolverAcc_RAC::TreeIkSolverAcc_RAC(const Tree& tree)
  : super(tree), jnt2jac_(tree_), jnt2jdqd_(tree_), jntparser_(tree_)
{
  updateInternalDataStructures();
}

void TreeIkSolverAcc_RAC::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  jnt2jac_.updateInternalDataStructures();
  jnt2jdqd_.updateInternalDataStructures();
  jntparser_.updateInternalDataStructures();

  qdd_min_.conservativeResize(nj_);
  qdd_max_.conservativeResize(nj_);

  qp_solver_.x_scale = VectorXd::Ones(nj_);
  qp_solver_.problem.G.conservativeResize(0, nj_);
  qp_solver_.problem.h.conservativeResize(0);
}

int TreeIkSolverAcc_RAC::CartToJnt(const JntArray& q_in, const JntArray& qd_in, const AccelMap& acc_in)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q_in.rows() != nj_ || qd_in.rows())
    return setDefaultError(E_SIZE_MISMATCH);

  const auto num_points = acc_in.size();
  const auto eq_dim = 6 * num_points;

  // Update Jdqd
  if (jnt2jdqd_.JntToCart(q_in, qd_in) < 0)
    return copyError(jnt2jdqd_);

  // Create big jacobian and acceleration
  J_.conservativeResize(eq_dim, nj_);
  a_.conservativeResize(eq_dim);
  size_t i = 0;
  for (const auto& [seg_name, accel] : acc_in)
  {
    // Update big jacobian
    if (jnt2jac_.JntToJac(q_in, seg_name) < 0)
      return copyError(jnt2jac_);
    J_.block(6 * i, 0, 6, nj_) = jnt2jac_.getJacobian().data;

    // Update big acceleration
    const auto& Jdqd = jnt2jdqd_.getJdqd(seg_name);
    a_.segment(6 * i, 3) = (accel.linear - Jdqd.linear).data;
    a_.segment(6 * i + 3, 3) = (accel.angular - Jdqd.angular).data;

    ++i;
  }

  // 評価関数
  const VectorXd Wt = eigen_tools::tile(Wt_, num_points, 0);
  const VectorXd Wj = VectorXd::Constant(nj_, Wj_);
  const MatrixXd JT_Wt = J_.transpose() * Wt.asDiagonal();
  qp_solver_.problem.P = JT_Wt * J_;
  qp_solver_.problem.P.diagonal() += Wj;
  qp_solver_.problem.q = -JT_Wt * a_;

  // 不等式制約
  qdd_min_.fill(numeric_limits<double>::lowest());
  qdd_max_.fill(numeric_limits<double>::max());
  for (size_t j = 0; j < nj_; ++j)
  {
    // 既に関節角制限をオーバーしている場合は，それ以上違反量を大きくしないように制限
    if (q_in(j) < jntparser_.lowerLimit(j))
      qdd_min_(j) = 0.;
    else if (q_in(j) > jntparser_.upperLimit(j))
      qdd_max_(j) = 0.;
  }
  quadprog::matIneqFromRange(qdd_min_, qdd_max_, qp_solver_.problem.A, qp_solver_.problem.b);

  // QPを解く
  if (!qp_solver_.solve())
    return setDefaultError(E_QP_FAILED);
  qdd_out_.data = qp_solver_.solution();

  return setDefaultError(E_NOERROR);
}

bool TreeIkSolverAcc_RAC::setWeightTS(const Vector6d& Wt)
{
  if ((Wt.array() < 0).any())
    return false;

  Wt_ = Wt;
  return true;
}

const Vector6d& TreeIkSolverAcc_RAC::getWeightTS() const
{
  return Wt_;
}

bool TreeIkSolverAcc_RAC::setWeightJS(const double& Wj)
{
  // 数値エラーを防ぐため正則化項は必ず入れる
  if (Wj <= 0)
    return false;

  Wj_ = Wj;
  return true;
}

const double& TreeIkSolverAcc_RAC::getWeightJS() const
{
  return Wj_;
}
}  // namespace kdl
