#include "../include/tobas_kdl/chainjnttoinertiasolver.hpp"

using namespace std;

namespace kdl
{
ChainJntToInertiaSolver::ChainJntToInertiaSolver(const Chain& chain) : super(chain)
{
  updateInternalDataStructures();
}

void ChainJntToInertiaSolver::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  I_.resize(ns_);
  X_.resize(ns_);
}

int ChainJntToInertiaSolver::JntToCart(const JntArray& q)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  size_t j = 0;
  double qj;

  // Sweep from root to leaf
  for (size_t i = 0; i < ns_; ++i)
  {
    I_[i] = chain_.getSegment(i).getInertia();
    if (chain_.getSegment(i).getJoint().type != Joint::Fixed)
    {
      qj = q(j);
      ++j;
    }
    else
    {
      qj = 0.;
    }
    X_[i] = chain_.getSegment(i).pose(qj);
  }

  // Sweep from leaf to root
  for (int i = ns_ - 1; i > 0; --i)
    I_[i - 1] = I_[i - 1] + X_[i] * I_[i];

  // 最後に{root}座標系に変換して返す
  I_out_ = X_[0] * I_[0];

  return setDefaultError(E_NOERROR);
}
}  // namespace kdl
