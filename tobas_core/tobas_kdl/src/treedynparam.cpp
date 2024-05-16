#include "../include/tobas_kdl/treedynparam.hpp"

using namespace std;

namespace tobas_kdl
{
TreeDynParam::TreeDynParam(const Tree& tree, const Vector& grav)
  : super(tree), rne_coriolis_(tree_, tobas_kdl::Vector::Zero()), rne_gravity_(tree_, grav)
{
  updateInternalDataStructures();
}

void TreeDynParam::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  rne_coriolis_.updateInternalDataStructures();
  rne_gravity_.updateInternalDataStructures();

  jntarray_null_ = JntArray::Zero(nj_);
}

int TreeDynParam::JntToCoriolis(const JntArray& q, const JntArray& qd)
{
  rne_coriolis_.CartToJnt(q, qd, jntarray_null_);
  return copyError(rne_coriolis_);
}

int TreeDynParam::JntToGravity(const JntArray& q)
{
  rne_gravity_.CartToJnt(q, jntarray_null_, jntarray_null_);
  return copyError(rne_gravity_);
}
}  // namespace tobas_kdl
