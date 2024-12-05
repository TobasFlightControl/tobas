#include "../include/tobas_kdl/chainidsolver_rne.hpp"

namespace kdl
{
ChainIdSolver_RNE::ChainIdSolver_RNE(const Chain& chain) : super(chain), ag_(Accel::Zero())
{
  updateInternalDataStructures();
}

void ChainIdSolver_RNE::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  X_.resize(ns_);
  S_.resize(ns_);
  v_.resize(ns_);
  a_.resize(ns_);
  f_.resize(ns_);

  effort_out_ = JntArray::Zero(nj_);
}

int ChainIdSolver_RNE::CartToJnt(
  const JntArray& q,
  const JntArray& qd,
  const JntArray& qdd,
  const Wrenches& forces,
  const Vector& grav)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || qd.rows() != nj_ || qdd.rows() != nj_ || forces.size() != ns_)
    return setDefaultError(E_SIZE_MISMATCH);

  // Update gravity
  ag_.linear = -grav;

  // Sweep from root to leaf
  j_ = 0;
  for (size_t i = 0; i < ns_; ++i)
  {
    if (chain_.getSegment(i).joint().type != Joint::FIXED)
    {
      qj_ = q(j_);
      qdj_ = qd(j_);
      qddj_ = qdd(j_);
      ++j_;
    }
    else
    {
      qj_ = 0.;
      qdj_ = 0.;
      qddj_ = 0.;
    }

    X_[i] = chain_.getSegment(i).pose(qj_);
    S_[i] = X_[i].M.inverse(chain_.getSegment(i).jacobian(qj_));
    const auto vj = X_[i].M.inverse(chain_.getSegment(i).twist(qj_, qdj_));
    if (i == 0)
    {
      v_[i] = vj;
      a_[i] = X_[i].inverse(ag_) + S_[i].accel(qddj_) + vj * vj;
    }
    else
    {
      v_[i] = X_[i].inverse(v_[i - 1]) + vj;
      a_[i] = X_[i].inverse(a_[i - 1]) + S_[i].accel(qddj_) + v_[i] * vj;
    }
    const auto Ii = chain_.getSegment(i).inertia();
    f_[i] = Ii * a_[i] + v_[i] * (Ii * v_[i]) - forces[i];
  }

  // Sweep from leaf to root
  j_ = nj_ - 1;
  for (int i = ns_ - 1; i >= 0; --i)
  {
    if (chain_.getSegment(i).joint().type != Joint::FIXED)
      effort_out_(j_--) = S_[i].dot(f_[i]);
    if (i != 0)
      f_[i - 1] = f_[i - 1] + X_[i] * f_[i];
  }

  return setDefaultError(E_NOERROR);
}

int ChainIdSolver_RNE::CartToJnt(
  const JntArray& q,
  const JntArray& qd,
  const JntArray& qdd,
  const Wrench& f_ee,
  const Vector& grav)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || qd.rows() != nj_ || qdd.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  // Update gravity
  ag_.linear = -grav;

  // Sweep from root to leaf
  for (size_t i = 0; i < ns_; ++i)
  {
    if (chain_.getSegment(i).joint().type != Joint::FIXED)
    {
      qj_ = q(j_);
      qdj_ = qd(j_);
      qddj_ = qdd(j_);
      ++j_;
    }
    else
    {
      qj_ = 0.;
      qdj_ = 0.;
      qddj_ = 0.;
    }

    X_[i] = chain_.getSegment(i).pose(qj_);
    const auto vj = X_[i].M.inverse(chain_.getSegment(i).twist(qj_, qdj_));
    S_[i] = X_[i].M.inverse(chain_.getSegment(i).jacobian(qj_));
    if (i == 0)
    {
      v_[i] = vj;
      a_[i] = X_[i].inverse(ag_) + S_[i].accel(qddj_) + v_[i] * vj;
      f_ee_ = X_[i].M.inverse(f_ee);  // CHANGED
    }
    else
    {
      v_[i] = X_[i].inverse(v_[i - 1]) + vj;
      a_[i] = X_[i].inverse(a_[i - 1]) + S_[i].accel(qddj_) + v_[i] * vj;
      f_ee_ = X_[i].M.inverse(f_ee_);  // CHANGED
    }
    const auto& Ii = chain_.getSegment(i).inertia();
    /* -----CHANGED----- */
    // f_[i] = Ii * a_[i] + v_[i] * (Ii * v_[i]) - forces[i];
    f_[i] = Ii * a_[i] + v_[i] * (Ii * v_[i]);
    if (i == ns_ - 1)
      f_[i] -= f_ee_;
    /* -----CHANGED----- */
  }

  // Sweep from leaf to root
  j_ = nj_ - 1;
  for (int i = ns_ - 1; i >= 0; --i)
  {
    if (chain_.getSegment(i).joint().type != Joint::FIXED)
      effort_out_(j_--) = S_[i].dot(f_[i]);
    if (i != 0)
      f_[i - 1] = f_[i - 1] + X_[i] * f_[i];
  }

  return setDefaultError(E_NOERROR);
}
}  // namespace kdl
