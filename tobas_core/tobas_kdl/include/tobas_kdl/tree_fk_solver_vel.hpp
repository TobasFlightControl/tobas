#pragma once

#include "./frame_vel.hpp"
#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace kdl
{
class TreeFkSolverVel : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeFkSolverVel(const Tree& tree);

  int JntToCart(const JntArray& q, const JntArray& qd, const std::string& seg_name);

  const FrameVel& getFrameVel() const
  {
    return p_out_;
  }

private:
  FrameVel p_out_;

  FrameVel recursiveFk(const JntArray& q, const JntArray& qd, const SegmentMap::const_iterator& seg_it);
};
}  // namespace kdl
