#include "../include/tobas_kdl/chaindynparam.hpp"

using namespace std;

namespace KDL
{
ChainDynParam::ChainDynParam(const Chain& chain)
  : super(chain), rne_coriolis_(chain_), rne_gravity_(chain_)
{
  updateInternalDataStructures();
}

void ChainDynParam::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  rne_coriolis_.updateInternalDataStructures();
  rne_gravity_.updateInternalDataStructures();

  zero_jntarray_ = JntArray::Zero(nj_);
  zero_wrenches_.resize(ns_, Wrench::Zero());
}

int ChainDynParam::JntToCoriolis(const JntArray& q, const JntArray& qd)
{
  rne_coriolis_.CartToJnt(q, qd, zero_jntarray_, zero_wrenches_, zero_vector_);
  return copyError(rne_coriolis_);
}

int ChainDynParam::JntToGravity(const JntArray& q, const Vector& grav)
{
  rne_gravity_.CartToJnt(q, zero_jntarray_, zero_jntarray_, zero_wrenches_, grav);
  return copyError(rne_gravity_);
}
}  // namespace KDL
