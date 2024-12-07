#pragma once

#include "./tree_solver_i.hpp"
#include "./frames.hpp"
#include "./jntarray.hpp"

namespace kdl
{
/* 全てのフレームの位置を一度に計算する． */
class TreeFkSolverPosAll : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeFkSolverPosAll(const Tree& tree);

  bool updateInternalDataStructures() override;

  int JntToCart(const JntArray& q);

  const Frame& getFrame(const std::string& seg_name) const
  {
    return frames_.at(seg_name);
  }

  const FrameMap& getFrames() const
  {
    return frames_;
  }

private:
  FrameMap frames_;

  void recursiveFk(const JntArray& q, const Frame& par_frame, const SegmentMap::const_iterator& cur_it);
};
}  // namespace kdl
