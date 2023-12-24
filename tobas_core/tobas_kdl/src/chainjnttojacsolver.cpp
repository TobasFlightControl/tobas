#include "../include/tobas_kdl/chainjnttojacsolver.hpp"

using namespace std;

namespace KDL
{
ChainJntToJacSolver::ChainJntToJacSolver(const Chain& chain) : super(chain)
{
  updateInternalDataStructures();
}

void ChainJntToJacSolver::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  locked_joints_.resize(nj_, false);
  jac_out_.resize(nj_);
}

bool ChainJntToJacSolver::setLockedJoints(const vector<bool> locked_joints)
{
  if (locked_joints.size() != locked_joints_.size())
    return false;

  locked_joints_ = locked_joints;
  return true;
}

int ChainJntToJacSolver::JntToJac(const JntArray& q_in, int _seg_nr)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q_in.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);

  const size_t seg_nr = _seg_nr >= 0 ? _seg_nr : chain_.getNrOfSegments();
  if (seg_nr > chain_.getNrOfSegments())
    return setDefaultError(E_OUT_OF_RANGE);

  // Initialize Jacobian to zero since only seg_nr columns are computed
  setToZero(jac_out_);

  Frame T_tmp = Frame::Identity();
  SegmentJacobian t_tmp = SegmentJacobian::Zero();
  size_t j = 0;
  size_t k = 0;
  for (size_t i = 0; i < seg_nr; ++i)
  {
    // Calculate new Frame_base_ee
    if (chain_.getSegment(i).getJoint().type != Joint::Fixed)
    {
      // pose of the new end-point expressed in the base
      total_ = T_tmp * chain_.getSegment(i).pose(q_in(j));
      // changing base of new segment's twist to base frame if it is not locked
      // t_tmp = T_tmp.M*chain_.getSegment(i).twist(1.0);
      if (!locked_joints_[j])
        t_tmp = T_tmp.M * chain_.getSegment(i).jacobian(q_in(j));
    }
    else
    {
      total_ = T_tmp * chain_.getSegment(i).pose(0);
    }

    // Changing Refpoint of all columns to new ee
    changeRefPoint(jac_out_, total_.p - T_tmp.p, jac_out_);

    // Only increase jointnr if the segment has a joint
    if (chain_.getSegment(i).getJoint().type != Joint::Fixed)
    {
      // Only put the twist inside if it is not locked
      if (!locked_joints_[j])
        jac_out_.setColumn(k++, t_tmp);
      ++j;
    }

    T_tmp = total_;
  }
  return setDefaultError(E_NOERROR);
}
}  // namespace KDL
