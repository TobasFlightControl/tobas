#include <iostream>
#include <eigen3/Eigen/SVD>

#include "../include/tobas_kdl/chainiksolveracc_rac.hpp"

using namespace std;
using namespace Eigen;

namespace kdl
{
ChainIkSolverAcc_RAC::ChainIkSolverAcc_RAC(const Chain& chain) : super(chain), jnt2jac_(chain_), jnt2jdqd_(chain_)
{
}

void ChainIkSolverAcc_RAC::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  jnt2jac_.updateInternalDataStructures();
  jnt2jdqd_.updateInternalDataStructures();
}

int ChainIkSolverAcc_RAC::CartToJnt(const JntArray& q, const JntArray& qd, const Vector& acc_ref)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || qd.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  // 目標加速度を計算
  if (jnt2jdqd_.JntToCart(q, qd) < 0)
    return copyError(jnt2jdqd_);
  const auto acc_diff = acc_ref - jnt2jdqd_.getJdqd().linear;

  // ヤコビアンを更新
  if (jnt2jac_.JntToJac(q) < 0)
    return copyError(jnt2jac_);
  const auto& jac = jnt2jac_.getJacobian();

  // 最小二乗解を計算
  // TODO: eigen_tools::minimizeWeightedNorm
  qdd_out_.data = jac.data.topRows(3).jacobiSvd(ComputeThinU | ComputeThinV).solve(acc_diff.data);

  return setDefaultError(E_NOERROR);
}

int ChainIkSolverAcc_RAC::CartToJnt(const JntArray& q, const JntArray& qd, const Accel& acc_ref)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || qd.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  // 目標加速度を計算
  if (jnt2jdqd_.JntToCart(q, qd) < 0)
    return copyError(jnt2jdqd_);
  const auto acc_diff = acc_ref - jnt2jdqd_.getJdqd();
  const auto acc_diff_ravel = acc_diff.ravel();

  // ヤコビアンを更新
  if (jnt2jac_.JntToJac(q) < 0)
    return copyError(jnt2jac_);
  const auto& jac = jnt2jac_.getJacobian();

  // 最小二乗解を計算
  // TODO: eigen_tools::minimizeWeightedNorm
  qdd_out_.data = jac.data.jacobiSvd(ComputeThinU | ComputeThinV).solve(acc_diff_ravel);

  return setDefaultError(E_NOERROR);
}
}  // namespace kdl
