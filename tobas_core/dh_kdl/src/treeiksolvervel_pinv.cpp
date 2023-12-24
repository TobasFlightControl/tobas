#include <dh_std_tools/unordered_set.hpp>
#include <dh_eigen_tools/linalg.hpp>

#include "../include/dh_kdl/treeiksolvervel_pinv.hpp"

using namespace std;
using namespace Eigen;

namespace KDL
{
TreeIkSolverVel_pinv::TreeIkSolverVel_pinv(const Tree& tree) : super(tree), jnt2jac_(tree_)
{
}

void TreeIkSolverVel_pinv::updateInternalDataStructures()
{
  super::updateInternalDataStructures();
  jnt2jac_.updateInternalDataStructures();
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

  // 最小二乗解を計算
  const VectorXd Wt = eigen_tools::tile(Wt_, num_points, 0);
  const VectorXd Wj = VectorXd::Constant(nj_, Wj_);
  qd_out_.data = eigen_tools::minimizeWeightedNorm<double, Dynamic, Dynamic>(J, t, Wt, Wj);

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
