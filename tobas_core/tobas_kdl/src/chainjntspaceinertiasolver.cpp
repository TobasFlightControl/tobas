#include "../include/tobas_kdl/chainjntspaceinertiasolver.hpp"

using namespace std;

namespace kdl
{
ChainJntSpaceInertiaSolver::ChainJntSpaceInertiaSolver(const Chain& chain) : super(chain)
{
  updateInternalDataStructures();
}

void ChainJntSpaceInertiaSolver::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  I_.resize(ns_);
  X_.resize(ns_);
  S_.resize(ns_);
  H_out_.resize(nj_);
}

int ChainJntSpaceInertiaSolver::JntToMass(const JntArray& q)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  // Sweep from root to leaf
  k_ = 0;
  for (size_t i = 0; i < ns_; ++i)
  {
    I_[i] = chain_.getSegment(i).inertia();
    if (chain_.getSegment(i).joint().type != Joint::Fixed)
    {
      qk_ = q(k_);
      ++k_;
    }
    else
    {
      qk_ = 0.;
    }
    X_[i] = chain_.getSegment(i).pose(qk_);
    S_[i] = X_[i].M.inverse(chain_.getSegment(i).jacobian(qk_));
  }

  // Sweep from leaf to root
  k_ = nj_ - 1;
  for (int i = ns_ - 1; i >= 0; --i)
  {
    if (i != 0)
    {
      I_[i - 1] = I_[i - 1] + X_[i] * I_[i];
    }
    auto F = I_[i] * S_[i];
    if (chain_.getSegment(i).joint().type != Joint::Fixed)
    {
      H_out_(k_, k_) = S_[i].dot(F);
      int j = k_;
      int l = i;
      while (l != 0)
      {
        F = X_[l] * F;
        --l;
        if (chain_.getSegment(l).joint().type != Joint::Fixed)
        {
          --j;
          H_out_(k_, j) = S_[l].dot(F);
          H_out_(j, k_) = H_out_(k_, j);
        }
      }
      --k_;
    }
  }

  return setDefaultError(E_NOERROR);
}
}  // namespace kdl
