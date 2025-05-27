#include "tobas_kdl/chain_fk_solver_pos.hpp"

using namespace std;

namespace kdl
{
ChainFkSolverPos::ChainFkSolverPos(const Chain& chain) : super(chain)
{
}

int ChainFkSolverPos::JntToCart(const JntArray& q_in, int _seg_nr)
{
  if (!isUpToDate()) {
    return setDefaultError(E_NOT_UP_TO_DATE);
  }
  if (q_in.rows() != nj_) {
    return setDefaultError(E_SIZE_MISMATCH);
  }

  const size_t seg_nr = _seg_nr >= 0 ? _seg_nr : chain_.getNrOfSegments();
  if (seg_nr > chain_.getNrOfSegments()) {
    return setDefaultError(E_OUT_OF_RANGE);
  }

  p_out_.setIdentity();
  j_ = 0;

  for (size_t i = 0; i < seg_nr; ++i) {
    const auto& seg = chain_.getSegment(i);
    const auto qj = seg.joint().type != Joint::FIXED ? q_in(j_++) : 0.;
    p_out_ = p_out_ * seg.pose(qj);
  }

  return setDefaultError(E_NOERROR);
}
}  // namespace kdl
