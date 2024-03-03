#include <iostream>

#include "../include/dh_kdl/treekdlmodel.hpp"

using namespace std;

namespace KDL
{
TreeKDLModel::TreeKDLModel(const Tree& tree, const Vector& grav)
  : tree_(tree),
    jnt_parser_(tree_),
    jnt2jac_(tree_),
    fk_pos_(tree_),
    fk_vel_(tree_),
    rne_(tree_, grav),
    dynparam_(tree_, grav),
    inertia_solver_(tree_)
{
}

void TreeKDLModel::updateInternalDataStructures()
{
  jnt_parser_.updateInternalDataStructures();
  jnt2jac_.updateInternalDataStructures();
  fk_pos_.updateInternalDataStructures();
  fk_vel_.updateInternalDataStructures();
  rne_.updateInternalDataStructures();
  dynparam_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
}

size_t TreeKDLModel::getNrOfJoints()
{
  return tree_.getNrOfJoints();
}

size_t TreeKDLModel::getNrOfSegments()
{
  return tree_.getNrOfSegments();
}

void TreeKDLModel::segmentJacobian(const JntArray& q, const string& name, Jacobian& jac)
{
  int res = jnt2jac_.JntToJac(q, jac, name);
  assert(res == 0);
}

Frame TreeKDLModel::fkPos(const JntArray& q, const string& name)
{
  return fk_pos_.JntToCart(q, name);
}

FrameVel TreeKDLModel::fkVel(const JntArray& q, const JntArray& qd, const string& name)
{
  return fk_vel_.JntToCart(q, qd, name);
}

void TreeKDLModel::jntSpaceInertiaMatrix(
  const JntArray& q,
  const JntArray& qd,
  JntSpaceInertiaMatrix& inertia)
{
  return dynparam_.JntToMass(q, qd, inertia);
}

void TreeKDLModel::inverseDynamics(
  const JntArray& q,
  const JntArray& qd,
  const JntArray& qdd,
  const WrenchMap& f_ext,
  JntArray& torques)
{
  int res = rne_.CartToJnt(q, qd, qdd, f_ext, torques);
  assert(res == 0);
}

void TreeKDLModel::inverseDynamics(
  const JntArray& q,
  const JntArray& qd,
  const JntArray& qdd,
  JntArray& torques)
{
  inverseDynamics(q, qd, qdd, wrenchmap_null_, torques);
}

void TreeKDLModel::jntSpaceInertiaMatrix(const JntArray& q, JntSpaceInertiaMatrix& inertia)
{
  return dynparam_.JntToMass(q, inertia);
}

void TreeKDLModel::coriolisEffort(const JntArray& q, const JntArray& qd, JntArray& torque)
{
  return dynparam_.JntToCoriolis(q, qd, torque);
}

void TreeKDLModel::gravityEffort(const JntArray& q, JntArray& torque)
{
  return dynparam_.JntToGravity(q, torque);
}

RigidBodyInertia TreeKDLModel::treeInertia(const JntArray& q_in)
{
  return inertia_solver_.JntToCart(q_in);
}

double TreeKDLModel::treeMass()
{
  return inertia_solver_.JntToMass();
}

const vector<string>& TreeKDLModel::jointNames() const
{
  return jnt_parser_.jointNames();
}

const unordered_map<string, size_t>& TreeKDLModel::jointIndexMap() const
{
  return jnt_parser_.jointIndexMap();
}

const string& TreeKDLModel::jointName(const int& idx) const
{
  return jnt_parser_.jointName(idx);
}

const size_t& TreeKDLModel::jointIndex(const string& name) const
{
  return jnt_parser_.jointIndex(name);
}
}  // namespace KDL
