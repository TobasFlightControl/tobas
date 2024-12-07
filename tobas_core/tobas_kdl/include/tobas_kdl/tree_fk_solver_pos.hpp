#pragma once

#include "./tree_solver_i.hpp"
#include "./frames.hpp"
#include "./jntarray.hpp"

namespace kdl
{
class TreeFkSolverPos : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeFkSolverPos(const Tree& tree);

  int JntToCart(const JntArray& q, const std::string& seg_name);

  const Frame& getFrame() const
  {
    return p_out_;
  }

private:
  Frame p_out_;

  Frame recursiveFk(const JntArray& q, const SegmentMap::const_iterator& seg_it);
};
}  // namespace kdl
