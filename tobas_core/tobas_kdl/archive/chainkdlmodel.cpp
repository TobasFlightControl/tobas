#include <iostream>

#include <dh_std_tools/unordered_map.hpp>

#include "../include/tobas_kdl/chainkdlmodel.hpp"

using namespace std;

namespace KDL
{
ChainKDLModel::ChainKDLModel(const Chain& chain)
  : chain_(chain),
    nj_(chain_.getNrOfJoints()),
    ns_(chain_.getNrOfSegments()),
    jnt2jac_(chain_),
    fk_pos_(chain_),
    fk_vel_(chain_),
    ik_vel_(chain_),
    ik_acc_(chain_),
    ext_rne_(chain_),
    ext_dynparam_(chain_),
    inertia_solver_(chain_)
{
  parseJntNames();
}

void ChainKDLModel::updateInternalDataStructures()
{
  nj_ = chain_.getNrOfJoints();
  ns_ = chain_.getNrOfSegments();

  jnt2jac_.updateInternalDataStructures();
  fk_pos_.updateInternalDataStructures();
  fk_vel_.updateInternalDataStructures();
  ik_vel_.updateInternalDataStructures();
  ik_acc_.updateInternalDataStructures();
  ext_rne_.updateInternalDataStructures();
  ext_dynparam_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();

  parseJntNames();
}

void ChainKDLModel::segmentJacobian(const JntArray& q, const string& name, Jacobian& jac)
{
  int res = jnt2jac_.JntToJac(q, jac, getSegIdx(name) + 1);
  assert(res == 0);
}

void ChainKDLModel::fkPos(const JntArray& q, const string& name, Frame& frame)
{
  int res = fk_pos_.JntToCart(q, frame, getSegIdx(name) + 1);
  assert(res == 0);
}

void ChainKDLModel::fkVel(const JntArrayVel& q, const string& name, FrameVel& frame)
{
  int res = fk_vel_.JntToCart(q, frame, getSegIdx(name) + 1);
  assert(res == 0);
}

void ChainKDLModel::ikVel(const Vector& v, const JntArray& q, JntArray& qd)
{
  return ik_vel_.CartToJnt(q, v, qd);
}

void ChainKDLModel::ikVel(const Twist& v, const JntArray& q, JntArray& qd)
{
  return ik_vel_.CartToJnt(q, v, qd);
}

void ChainKDLModel::ikAcc(const Vector& a, const JntArray& q, const JntArray& qd, JntArray& qdd)
{
  return ik_acc_.CartToJnt(a, q, qd, qdd);
}

void ChainKDLModel::ikAcc(const Twist& a, const JntArray& q, const JntArray& qd, JntArray& qdd)
{
  return ik_acc_.CartToJnt(a, q, qd, qdd);
}

void ChainKDLModel::calcJdqd(const JntArray& q, const JntArray& qd, Twist& Jdqd)
{
  return ik_acc_.calcJdqd(q, qd, Jdqd);
}

const RigidBodyInertia& ChainKDLModel::segmentInertia(const string& name)
{
  return chain_.getSegment(getSegIdx(name)).getInertia();
}

RigidBodyInertia ChainKDLModel::chainInertia(const JntArray& q)
{
  return inertia_solver_.JntToCart(q);
}

void ChainKDLModel::inverseDynamics(
  const JntArray& q,
  const JntArray& qd,
  const JntArray& qdd,
  const Wrenches& forces,
  const Vector& grav,
  JntArray& torques)
{
  return ext_rne_.CartToJnt(q, qd, qdd, forces, grav, torques);
}

void ChainKDLModel::inverseDynamics(
  const JntArray& q,
  const JntArray& qd,
  const JntArray& qdd,
  const Wrench& f_ee,
  const Vector& grav,
  JntArray& torques)
{
  return ext_rne_.CartToJnt(q, qd, qdd, f_ee, grav, torques);
}

void ChainKDLModel::jntSpaceInertiaMatrix(const JntArray& q, JntSpaceInertiaMatrix& inertia)
{
  return ext_dynparam_.JntToMass(q, inertia);
}

void ChainKDLModel::coriolisEffort(const JntArray& q, const JntArray& qd, JntArray& torque)
{
  return ext_dynparam_.JntToCoriolis(q, qd, torque);
}

void ChainKDLModel::gravityEffort(const JntArray& q, const Vector& grav, JntArray& torque)
{
  return ext_dynparam_.JntToGravity(q, grav, torque);
}

const vector<string>& ChainKDLModel::jntNames() const
{
  return jnt_names_;
}

void ChainKDLModel::parseJntNames()
{
  for (size_t i = 0; i < chain_.getNrOfSegments(); ++i)
  {
    const Segment& segment = chain_.getSegment(i);  // 0でrootの次のフレームが返ってくる
    const Joint& joint = segment.getJoint();
    const string& segment_name = segment.name();

    seg2idx_[segment_name] = i;  // rootの次のフレームのインデックスを1にする
    if (joint.type != Joint::None)
      jnt_names_.push_back(joint.name);
  }
}

int ChainKDLModel::getSegIdx(const string& name)
{
  assert(dh_std::contains(seg2idx_, name));
  int idx = seg2idx_[name];
  assert(chain_.getSegment(idx).name() == name);
  return idx;
}
}  // namespace KDL
