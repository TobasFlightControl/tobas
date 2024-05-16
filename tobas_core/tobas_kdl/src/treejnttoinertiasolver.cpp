#include "../include/tobas_kdl/treejnttoinertiasolver.hpp"

using namespace std;

namespace tobas_kdl
{
TreeJntToInertiaSolver::TreeJntToInertiaSolver(const Tree& tree) : super(tree)
{
  updateInternalDataStructures();
}

void TreeJntToInertiaSolver::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  jntarray_null_ = JntArray::Zero(nj_);

  const SegmentMap& segments = tree_.getSegments();
  for (SegmentMap::const_iterator seg = segments.begin(); seg != segments.end(); ++seg)
  {
    const string& seg_name = seg->first;
    X_[seg_name] = Frame();
    I_[seg_name] = RigidBodyInertia();
  }
}

int TreeJntToInertiaSolver::JntToCart(const JntArray& q_in)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q_in.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  auto root = tree_.getRootSegment();
  step(root, q_in);

  return setDefaultError(E_NOERROR);
}

void TreeJntToInertiaSolver::step(const SegmentMap::const_iterator& segment, const JntArray& q)
{
  const Segment& seg = segment->second.segment;
  const string& seg_name = segment->first;
  const string& par_name = segment->second.parent->first;

  // Forward calculation
  const size_t& j = segment->second.q_nr;
  const double& qj = seg.getJoint().type == Joint::Fixed ? 0. : q(j);
  X_.at(seg_name) = seg.pose(qj);
  I_.at(seg_name) = seg.getInertia();

  // Propagate calculations over each child segment
  SegmentMap::const_iterator child;
  for (size_t i = 0; i < segment->second.children.size(); ++i)
  {
    child = segment->second.children[i];
    step(child, q);
  }

  // Backward calculation
  if (segment != tree_.getRootSegment())
  {
    I_.at(par_name) = I_.at(par_name) + X_.at(seg_name) * I_.at(seg_name);
  }
}
}  // namespace tobas_kdl
