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

  for (const auto& [seg_name, _] : tree_.getSegments())
  {
    v_[seg_name] = Twist();
    a_[seg_name] = Accel();
    f_[seg_name] = Wrench();
  }

  effort_out_ = JntArray::Zero(nj_);
}

int TreeIdSolver_RNE::CartToJnt(
  const JntArray& q,
  const JntArray& q_dot,
  const JntArray& q_dotdot,
  const WrenchMap& f_ext)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q.rows() != nj_ || q_dot.rows() != nj_ || q_dotdot.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  // Do the recursion here
  rneStep(tree_.getRootSegment(), q, q_dot, q_dotdot, f_ext);

  return setDefaultError(E_NOERROR);
}

void TreeIdSolver_RNE::rneStep(
  const SegmentMap::const_iterator& segment,
  const JntArray& q,
  const JntArray& q_dot,
  const JntArray& q_dotdot,
  const WrenchMap& f_ext)
{
  const auto& seg = segment->second.segment;
  const auto& seg_name = segment->first;
  const auto& par_name = segment->second.parent->first;

  // Do forward calculations involving velocity & acceleration of this segment
  const auto& j = segment->second.q_nr;
  double qj, qdj, qddj;
  if (seg.getJoint().type != Joint::Fixed)
  {
    qj = q(j);
    qdj = q_dot(j);
    qddj = q_dotdot(j);
  }
  else
  {
    qj = qdj = qddj = 0;
  }

  const auto Xj = seg.pose(qj);
  const auto Sj = Xj.M.inverse(seg.jacobian(qj));    // Jacobian for current joint
  const auto vj = Xj.M.inverse(seg.twist(qj, qdj));  // Transform velocity

  // Calculate velocity and acceleration of the segment (in segment coordinates)
  if (segment == tree_.getRootSegment())
  {
    v_.at(seg_name) = vj;
    a_.at(seg_name) = Xj.inverse(ag_) + Sj.accel(qddj) + vj * vj;
  }
  else
  {
    v_.at(seg_name) = Xj.inverse(v_.at(par_name)) + vj;
    a_.at(seg_name) = Xj.inverse(a_.at(par_name)) + Sj.accel(qddj) + v_.at(seg_name) * vj;
  }

  // Calculate the force for the joint
  // Collect RigidBodyInertia and external forces
  const auto& I = seg.getInertia();
  f_.at(seg_name) = I * a_.at(seg_name) + v_.at(seg_name) * (I * v_.at(seg_name));
  if (f_ext.find(seg_name) != f_ext.end())
    f_.at(seg_name) = f_.at(seg_name) - f_ext.at(seg_name);

  // propagate calculations over each child segment
  for (const auto& child : segment->second.children)
    rneStep(child, q, q_dot, q_dotdot, f_ext);

  // Do backward calculations involving wrenches and joint efforts
  // If there is a moving joint, evaluate its effort
  if (seg.getJoint().type != Joint::Fixed)
  {
    effort_out_(j) = Sj.dot(f_.at(seg_name));
    // TODO: inertia, damping, frictionの補償をすべき？
  }

  // Add reaction forces to parent segment
  if (segment != tree_.getRootSegment())
    f_.at(par_name) = f_.at(par_name) + Xj * f_.at(seg_name);
}
}  // namespace kdl
