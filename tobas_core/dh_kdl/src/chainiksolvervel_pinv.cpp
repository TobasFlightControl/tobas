#include <iostream>
#include <Eigen/SVD>

#include "../include/dh_kdl/chainiksolvervel_pinv.hpp"

using namespace std;
using namespace Eigen;

namespace KDL
{
ChainIkSolverVel_pinv::ChainIkSolverVel_pinv(const Chain& chain) : super(chain), jnt2jac_(chain_)
{
  updateInternalDataStructures();
}

void ChainIkSolverVel_pinv::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  jnt2jac_.updateInternalDataStructures();
}

int ChainIkSolverVel_pinv::CartToJnt(const JntArray& q, const Vector& v)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  // ヤコビアンを更新
  if (jnt2jac_.JntToJac(q) < 0)
    return copyError(jnt2jac_);
  const auto& jac = jnt2jac_.getJacobian();

  // 最小二乗解を求める
  qd_out_.data = jac.data.topRows(3).jacobiSvd(ComputeThinU | ComputeThinV).solve(v.data);

  return setDefaultError(E_NOERROR);
}

int ChainIkSolverVel_pinv::CartToJnt(const JntArray& q, const Twist& v)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  // ヤコビアンを更新
  if (jnt2jac_.JntToJac(q) < 0)
    return copyError(jnt2jac_);
  const auto& jac = jnt2jac_.getJacobian();

  // 最小二乗解を求める
  const auto v_ravel = v.ravel();
  qd_out_.data = jac.data.jacobiSvd(ComputeThinU | ComputeThinV).solve(v_ravel);

  return setDefaultError(E_NOERROR);
}
}  // namespace KDL
