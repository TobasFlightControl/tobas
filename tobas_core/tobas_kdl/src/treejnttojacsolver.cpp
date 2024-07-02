#include <iostream>

#include "../include/tobas_kdl/treejnttojacsolver.hpp"

using namespace std;

namespace kdl
{
TreeJntToJacSolver::TreeJntToJacSolver(const Tree& tree) : super(tree)
{
  updateInternalDataStructures();
}

void TreeJntToJacSolver::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  J_out_.resize(nj_);
}

int TreeJntToJacSolver::JntToJac(const JntArray& q_in, const string& seg_name)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q_in.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  // Lets search the tree-element
  auto it = tree_.getSegment(seg_name);

  // If seg_name is not inside the tree, back out:
  if (it == tree_.getSegments().end())
    return setDefaultError(E_OUT_OF_RANGE);

  // Initialize
  J_out_.setZero();
  T_total_.setIdentity();

  // Lets recursively iterate until we are in the root segment
  const auto root = tree_.getRootSegment();
  while (it != root)
  {
    // get the corresponding q_nr for this TreeElement:
    const auto& q_nr = it->second.q_nr;

    // get the pose of the segment:
    const auto T_local = it->second.segment.pose(q_in(q_nr));
    // calculate new T_end:
    T_total_ = T_local * T_total_;

    // get the twist of the segment:
    if (it->second.segment.getJoint().type != Joint::Fixed)
    {
      auto t_local = it->second.segment.jacobian(q_in(q_nr));
      // transform the endpoint of the local twist to the global endpoint:
      t_local = t_local.refPoint(T_total_.p - T_local.p);
      // transform the base of the twist to the endpoint
      t_local = T_total_.M.inverse(t_local);
      // store the twist in the jacobian:
      J_out_.setColumn(q_nr, t_local);
    }

    // goto the parent
    it = it->second.parent;
  }

  // Change the base of the complete jacobian from the endpoint to the base
  changeBase(J_out_, T_total_.M, J_out_);

  return setDefaultError(E_NOERROR);
}
}  // namespace kdl
