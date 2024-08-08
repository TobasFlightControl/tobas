#include <stdexcept>

#include "../include/tobas_kdl/treeidsolver_rne.hpp"

using namespace std;

namespace kdl
{
TreeIdSolver_RNE::TreeIdSolver_RNE(const Tree& tree, const Vector& grav) : super(tree), ag_(-grav, Vector::Zero())
{
  updateInternalDataStructures();
}

void TreeIdSolver_RNE::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  for (const auto& [cur_name, _] : tree_.getSegments())
  {
    v_[cur_name] = Twist();
    a_[cur_name] = Accel();
    f_[cur_name] = Wrench();
  }

  effort_out_ = JntArray::Zero(nj_);
}

int TreeIdSolver_RNE::CartToJnt(const JntArray& q, const JntArray& qd, const JntArray& qdd, const WrenchMap& f_ext)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || qd.rows() != nj_ || qdd.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  // Do the recursion here
  rneStep(tree_.getRootSegment(), q, qd, qdd, f_ext);

  return setDefaultError(E_NOERROR);
}

void TreeIdSolver_RNE::rneStep(
  const SegmentMap::const_iterator& cur_it,
  const JntArray& q,
  const JntArray& qd,
  const JntArray& qdd,
  const WrenchMap& f_ext)
{
  const auto& cur_name = cur_it->first;
  const auto& cur_ele = cur_it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& par_it = cur_ele.parent;
  const auto& par_name = par_it->first;

  // Do forward calculations involving velocity & acceleration of this segment
  const auto& j = cur_ele.q_nr;
  if (cur_seg.joint().type != Joint::Fixed)
  {
    qj_ = q(j);
    qdj_ = qd(j);
    qddj_ = qdd(j);
  }
  else
  {
    qj_ = 0.;
    qdj_ = 0.;
    qddj_ = 0.;
  }

  const auto Xj = cur_seg.pose(qj_);
  const auto Sj = Xj.M.inverse(cur_seg.jacobian(qj_));     // Jacobian for current joint
  const auto vj = Xj.M.inverse(cur_seg.twist(qj_, qdj_));  // Transform velocity

  // Calculate velocity and acceleration of the segment (in segment coordinates)
  if (cur_it == tree_.getRootSegment())
  {
    v_.at(cur_name) = vj;
    a_.at(cur_name) = Xj.inverse(ag_) + Sj.accel(qddj_) + vj * vj;
  }
  else
  {
    v_.at(cur_name) = Xj.inverse(v_.at(par_name)) + vj;
    a_.at(cur_name) = Xj.inverse(a_.at(par_name)) + Sj.accel(qddj_) + v_.at(cur_name) * vj;
  }

  // Calculate the force for the joint
  // Collect RigidBodyInertia and external forces
  const auto& I = cur_seg.inertia();
  f_.at(cur_name) = I * a_.at(cur_name) + v_.at(cur_name) * (I * v_.at(cur_name));
  if (f_ext.find(cur_name) != f_ext.end())
    f_.at(cur_name) = f_.at(cur_name) - f_ext.at(cur_name);

  // propagate calculations over each child segment
  for (const auto& child : cur_ele.children)
    rneStep(child, q, qd, qdd, f_ext);

  // Do backward calculations involving wrenches and joint efforts
  // If there is a moving joint, evaluate its effort
  if (cur_seg.joint().type != Joint::Fixed)
  {
    effort_out_(j) = Sj.dot(f_.at(cur_name));
    // TODO: inertia, damping, frictionの補償をすべき？
  }

  // Add reaction forces to parent segment
  if (cur_it != tree_.getRootSegment())
    f_.at(par_name) = f_.at(par_name) + Xj * f_.at(cur_name);
}
}  // namespace kdl
