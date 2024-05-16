#include "../include/tobas_kdl/chainjntspaceinertiasolver.hpp"

using namespace std;

namespace KDL
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

  int k = 0;
  double qk;

  // Sweep from root to leaf
  for (size_t i = 0; i < ns_; ++i)
  {
    I_[i] = chain_.getSegment(i).getInertia();
    if (chain_.getSegment(i).getJoint().type != Joint::Fixed)
    {
      qk = q(k);
      ++k;
    }
    else
    {
      qk = 0.;
    }
    X_[i] = chain_.getSegment(i).pose(qk);
    S_[i] = X_[i].M.inverse(chain_.getSegment(i).jacobian(qk));
  }

  // Sweep from leaf to root
  k = nj_ - 1;
  for (int i = ns_ - 1; i >= 0; --i)
  {
    if (i != 0)
    {
      I_[i - 1] = I_[i - 1] + X_[i] * I_[i];
    }
    auto F = I_[i] * S_[i];
    if (chain_.getSegment(i).getJoint().type != Joint::Fixed)
    {
      H_out_(k, k) = S_[i].dot(F);
      int j = k;
      int l = i;
      while (l != 0)
      {
        F = X_[l] * F;
        --l;
        if (chain_.getSegment(l).getJoint().type != Joint::Fixed)
        {
          --j;
          H_out_(k, j) = S_[l].dot(F);
          H_out_(j, k) = H_out_(k, j);
        }
      }
      --k;
    }
  }

  return setDefaultError(E_NOERROR);
}
}  // namespace KDL
