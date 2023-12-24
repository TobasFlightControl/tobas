#include "../include/dh_kdl/treedynparam.hpp"

using namespace std;

namespace KDL
{
TreeDynParam::TreeDynParam(const Tree& tree, const Vector& grav)
  : super(tree),
    rne_mass_(tree_, vector_null_),
    rne_coriolis_(tree_, vector_null_),
    rne_gravity_(tree_, grav)
{
  updateInternalDataStructures();
}

void TreeDynParam::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  rne_mass_.updateInternalDataStructures();
  rne_coriolis_.updateInternalDataStructures();
  rne_gravity_.updateInternalDataStructures();

  jntarray_null_ = JntArray::Zero(nj_);

  elements_.resize(nj_, JntArray::Zero(nj_));
  for (size_t i = 0; i < nj_; ++i)
    elements_[i](i) = 1;

  H_out_.resize(nj_);
}

int TreeDynParam::JntToMass(const JntArray& q, const JntArray& qd)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || qd.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  if (rne_mass_.CartToJnt(q, qd, jntarray_null_, wrenchmap_null_) < 0)
    return copyError(rne_mass_);
  const auto bias = rne_mass_.getEfforts();  // 次で値が書き換わるためコピー

  for (size_t i = 0; i < nj_; ++i)
  {
    if (rne_mass_.CartToJnt(q, qd, elements_[i], wrenchmap_null_) < 0)
      return copyError(rne_mass_);
    const JntArray m = rne_mass_.getEfforts() - bias;
    for (size_t j = 0; j < nj_; ++j)
      H_out_(j, i) = m(j);
  }

  return setDefaultError(E_NOERROR);
}

int TreeDynParam::JntToMass(const JntArray& q)
{
  return JntToMass(q, jntarray_null_);
}

int TreeDynParam::JntToCoriolis(const JntArray& q, const JntArray& qd)
{
  rne_coriolis_.CartToJnt(q, qd, jntarray_null_, wrenchmap_null_);
  return copyError(rne_coriolis_);
}

int TreeDynParam::JntToGravity(const JntArray& q)
{
  rne_gravity_.CartToJnt(q, jntarray_null_, jntarray_null_, wrenchmap_null_);
  return copyError(rne_gravity_);
}
}  // namespace KDL
