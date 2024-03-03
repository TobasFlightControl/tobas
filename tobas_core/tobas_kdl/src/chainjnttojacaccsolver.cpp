#include "../include/tobas_kdl/chainjnttojacaccsolver.hpp"

namespace KDL
{
ChainJntToJacAccSolver::ChainJntToJacAccSolver(const Chain& chain) : super(chain)
{
  updateInternalDataStructures();
}

void ChainJntToJacAccSolver::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  X_.resize(ns_);
  v_.resize(ns_);
  a_.resize(ns_);
}

int ChainJntToJacAccSolver::JntToCart(const JntArray& q, const JntArray& qd)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || qd.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  size_t j = 0;
  double qj, qdj;

  for (size_t i = 0; i < ns_; ++i)
  {
    if (chain_.getSegment(i).getJoint().type != Joint::Fixed)
    {
      qj = q(j);
      qdj = qd(j);
      ++j;
    }
    else
    {
      qj = qdj = 0;
    }

    X_[i] = chain_.getSegment(i).pose(qj);  // X_[i] := {i - 1}から{i}への変換
    const auto vj = X_[i].M.inverse(chain_.getSegment(i).twist(qj, qdj));

    // {0}に対する(加)速度を各フレームから見たものを求める
    if (i == 0)
    {
      v_[i] = vj;
      a_[i] = vj * vj;
    }
    else
    {
      v_[i] = X_[i].inverse(v_[i - 1]) + vj;
      a_[i] = X_[i].inverse(a_[i - 1]) + v_[i] * vj;
    }
  }

  // {0}で表したものに変換する
  Jdqd_out_ = a_.back();
  for (int i = ns_ - 1; i >= 0; --i)
    Jdqd_out_ = X_[i].M * Jdqd_out_;

  return setDefaultError(E_NOERROR);
}
}  // namespace KDL
