#include "../include/tobas_kdl/treejntspacepid.hpp"

using namespace std;

namespace tobas_kdl
{
TreeJntSpacePID::TreeJntSpacePID(const Tree& tree, const Vector& grav)
  : super(tree), rne_(tree, grav)
{
  updateInternalDataStructures();
}

void TreeJntSpacePID::updateInternalDataStructures()
{
  super::updateInternalDataStructures();
  rne_.updateInternalDataStructures();
  zeros_ = JntArray::Zero(nj_);
}

int TreeJntSpacePID::CartToJnt(
  const JntArray& cur_q,
  const JntArray& cur_qd,
  const JntArray& tar_q,
  const JntArray& tar_qd,
  const JntArray& qdd_ff)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (
    cur_q.rows() != nj_ || cur_qd.rows() != nj_ || tar_q.rows() != nj_ || tar_qd.rows() != nj_
    || qdd_ff.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  // Compute target joint accelerations
  // TODO: I要素を加える
  const auto tar_qdd = qdd_ff + kp_ * (tar_q - cur_q) + kd_ * (tar_qd - cur_qd);

  // Compute target joint efforts
  if (rne_.CartToJnt(cur_q, cur_qd, tar_qdd) < 0)
    return copyError(rne_);

  return setDefaultError(E_NOERROR);
}

int TreeJntSpacePID::CartToJnt(
  const JntArray& cur_q,
  const JntArray& cur_qd,
  const JntArray& tar_q,
  const JntArray& tar_qd)
{
  return CartToJnt(cur_q, cur_qd, tar_q, tar_qd, zeros_);
}

bool TreeJntSpacePID::setStiffness(const double& kp)
{
  if (kp < 0)
    return false;

  kp_ = kp;
  return true;
}

bool TreeJntSpacePID::setDamping(const double& kd)
{
  if (kd < 0)
    return false;

  kd_ = kd;
  return true;
}
}  // namespace tobas_kdl
