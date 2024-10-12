#include <iostream>

#include <tobas_math/core.hpp>

#include "../include/tobas_kdl/chainiksolverpos_lm.hpp"
#include "../include/tobas_kdl/frames.hpp"

using namespace std;
using namespace Eigen;

namespace kdl
{
ChainIkSolverPos_LM::ChainIkSolverPos_LM(const Chain& chain) : super(chain)
{
  L_.head<3>().fill(kDefaultWeightPos);
  L_.tail<3>().fill(kDefaultWeightRot);

  updateInternalDataStructures();
}

void ChainIkSolverPos_LM::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  jac_.conservativeResize(NoChange, nj_);
  grad_.conservativeResize(nj_);
  T_base_jointroot_.resize(nj_);
  T_base_jointtip_.resize(nj_);
  svd_ = JacobiSVD<Matrix6Xd>(NoChange, nj_, ComputeThinU | ComputeThinV);
}

void ChainIkSolverPos_LM::displayJacobian(const JntArray& jval)
{
  const auto& q = jval.data;
  computeFwdPos(q);
  computeJacobian(q);
  svd_.compute(jac_);
  cout << "Singular values : " << svd_.singularValues().transpose() << endl;
}

int ChainIkSolverPos_LM::CartToJnt(const JntArray& q_init, const Frame& T_base_goal)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q_init.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  VectorXd q = q_init.data;
  computeFwdPos(q);
  Vector6d delta_pos = L_.asDiagonal() * (T_base_head_ - T_base_goal).toTwist().ravel();
  double delta_pos_norm = delta_pos.norm();
  if (delta_pos_norm < eps_cart_)
  {
    q_out_.data = q;
    return setDefaultError(E_NOERROR);
  }
  computeJacobian(q);
  jac_ = L_.asDiagonal() * jac_;

  auto lambda = kInitLambda;
  auto v = kInitV;
  for (size_t i = 0; i < max_iter_; ++i)
  {
    svd_.compute(jac_);
    VectorXd Aii = svd_.singularValues();
    for (Index j = 0; j < Aii.rows(); ++j)
      Aii(j) = Aii(j) / (Aii(j) * Aii(j) + lambda);
    const VectorXd diffq = svd_.matrixV() * Aii.cwiseProduct(svd_.matrixU().transpose() * delta_pos);
    grad_ = jac_.transpose() * delta_pos;
    const auto dnorm = diffq.lpNorm<Infinity>();
    if (dnorm < eps_jnt_)
    {
      q_out_.data = q;
      error_code_ = E_INCREMENT_JOINTS_TOO_SMALL;
      error_msg_ = "The joint position increments are to small";
      return error_code_;
    }

    if (grad_.transpose() * grad_ < eps_jnt_ * eps_jnt_)
    {
      q_out_.data = q;
      error_code_ = E_GRADIENT_JOINTS_TOO_SMALL;
      error_msg_ = "The gradient of E towards the joints is to small";
      return error_code_;
    }

    VectorXd q_new = q + diffq;
    enforceJointLimits(q_new);
    computeFwdPos(q_new);
    const Vector6d delta_pos_new = L_.asDiagonal() * (T_base_head_ - T_base_goal).toTwist().ravel();
    const auto delta_pos_new_norm = delta_pos_new.norm();
    auto rho = math::sqr(delta_pos_norm) - math::sqr(delta_pos_new_norm);
    rho /= diffq.transpose() * (lambda * diffq + grad_);
    if (rho > 0)
    {
      q = q_new;
      delta_pos = delta_pos_new;
      delta_pos_norm = delta_pos_new_norm;
      if (delta_pos_norm < eps_cart_)
      {
        q_out_.data = q;
        return setDefaultError(E_NOERROR);
      }
      computeJacobian(q_new);
      jac_ = L_.asDiagonal() * jac_;
      const auto tmp = 2 * rho - 1;
      lambda *= max(1 / 3., 1 - tmp * tmp * tmp);
      v = kInitV;
    }
    else
    {
      lambda *= v;
      v *= 2;
    }
  }
  q_out_.data = q;
  return setDefaultError(E_MAX_ITERATIONS_EXCEEDED);
}

bool ChainIkSolverPos_LM::setMaxIter(const size_t& max_iter)
{
  if (max_iter <= 0)
    return false;

  max_iter_ = max_iter;
  return true;
}

bool ChainIkSolverPos_LM::setEpsilonCart(const double& eps_cart)
{
  if (eps_cart < 0)
    return false;

  eps_cart_ = eps_cart;
  return true;
}

bool ChainIkSolverPos_LM::setEpsilonJnt(const double& eps_jnt)
{
  if (eps_jnt < 0)
    return false;

  eps_jnt_ = eps_jnt;
  return true;
}

bool ChainIkSolverPos_LM::setWeight(const Eigen::Vector6d& L)
{
  if ((L.array() < 0).any())
    return false;

  L_ = L;
  return true;
}

void ChainIkSolverPos_LM::computeFwdPos(const VectorXd& q)
{
  T_base_head_ = Frame::Identity();  // frame w.r.t. base of head
  size_t j = 0;                      // joint index
  for (size_t i = 0; i < ns_; ++i)
  {
    const auto& seg = chain_.getSegment(i);
    if (seg.joint().type != Joint::Fixed)
    {
      T_base_jointroot_[j] = T_base_head_;
      T_base_head_ = T_base_head_ * seg.pose(q(j));
      T_base_jointtip_[j] = T_base_head_;
      ++j;
    }
    else
    {
      T_base_head_ = T_base_head_ * seg.pose(0.);
    }
  }
}

void ChainIkSolverPos_LM::computeJacobian(const VectorXd& q)
{
  size_t j = 0;
  for (size_t i = 0; i < ns_; ++i)
  {
    const auto& seg = chain_.getSegment(i);
    if (seg.joint().type != Joint::Fixed)
    {
      // compute twist of the end effector motion caused by joint[j];
      // expressed in base frame, with vel. ref. point equal to the end effector
      const auto t = (T_base_jointroot_[j].M * seg.jacobian(q(j))).refPoint(T_base_head_.p - T_base_jointtip_[j].p);
      jac_.col(j) = t.ravel();
      ++j;
    }
  }
}

void ChainIkSolverPos_LM::enforceJointLimits(Eigen::VectorXd& q)
{
  size_t j = 0;
  for (size_t i = 0; i < ns_; ++i)
  {
    const auto& joint = chain_.getSegment(i).joint();
    if (joint.type != Joint::Fixed)
    {
      q(j) = clamp(q(j), joint.lower_limit, joint.upper_limit);
      ++j;
    }
  }
}
}  // namespace kdl
