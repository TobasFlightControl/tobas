#include <dh_eigen_tools/linalg.hpp>

#include "../include/dh_kdl/treeiksolveracc_rac.hpp"

using namespace std;
using namespace Eigen;

namespace KDL
{
TreeIkSolverAcc_RAC::TreeIkSolverAcc_RAC(const Tree& tree)
  : super(tree), jnt2jac_(tree_), jnt2jdqd_(tree_)
{
  updateInternalDataStructures();
}

void TreeIkSolverAcc_RAC::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  jnt2jac_.updateInternalDataStructures();
  jnt2jdqd_.updateInternalDataStructures();
}

int TreeIkSolverAcc_RAC::CartToJnt(
  const JntArray& q_in,
  const JntArray& qd_in,
  const AccelMap& acc_in)
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
  MatrixXd J(eq_dim, nj_);
  VectorXd a(eq_dim);
  size_t i = 0;
  for (const auto& [seg_name, accel] : acc_in)
  {
    // Update big jacobian
    if (jnt2jac_.JntToJac(q_in, seg_name) < 0)
      return copyError(jnt2jac_);
    J.block(6 * i, 0, 6, nj_) = jnt2jac_.getJacobian().data;

    // Update big acceleration
    const auto& Jdqd = jnt2jdqd_.getJdqd(seg_name);
    a.segment(6 * i, 3) = (accel.linear - Jdqd.linear).data;
    a.segment(6 * i + 3, 3) = (accel.angular - Jdqd.angular).data;

    ++i;
  }

  // 最小二乗解を計算
  const VectorXd Wt = eigen_tools::tile(Wt_, num_points, 0);
  const VectorXd Wj = VectorXd::Constant(nj_, Wj_);
  qdd_out_.data = eigen_tools::minimizeWeightedNorm<double, Dynamic, Dynamic>(J, a, Wt, Wj);

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
}  // namespace KDL
