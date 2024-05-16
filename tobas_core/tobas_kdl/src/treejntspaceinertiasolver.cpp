#include "../include/tobas_kdl/treejntspaceinertiasolver.hpp"

using namespace std;

namespace tobas_kdl
{
TreeJntSpaceInertiaSolver::TreeJntSpaceInertiaSolver(const Tree& tree)
  : super(tree), rne_(tree_, tobas_kdl::Vector::Zero())
{
  updateInternalDataStructures();
}

void TreeJntSpaceInertiaSolver::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  rne_.updateInternalDataStructures();

  elements_.resize(nj_, JntArray::Zero(nj_));
  for (size_t i = 0; i < nj_; ++i)
    elements_[i](i) = 1;

  H_out_.resize(nj_);
  jntarray_null_ = JntArray::Zero(nj_);
}

int TreeJntSpaceInertiaSolver::JntToMass(const JntArray& q)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || jntarray_null_.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  if (rne_.CartToJnt(q, jntarray_null_, jntarray_null_) < 0)
    return copyError(rne_);
  const auto bias = rne_.getEfforts();  // 次で値が書き換わるためコピー

  for (size_t i = 0; i < nj_; ++i)
  {
    if (rne_.CartToJnt(q, jntarray_null_, elements_[i]) < 0)
      return copyError(rne_);
    const auto m = rne_.getEfforts() - bias;
    for (size_t j = 0; j < nj_; ++j)
      H_out_(j, i) = m(j);
  }

  return setDefaultError(E_NOERROR);
}
}  // namespace tobas_kdl
