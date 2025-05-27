#pragma once

#include "./frames.hpp"
#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

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

  inline const Frame& getFrame(const std::string& seg_name) const;
  inline const FrameMap& getFrames() const;

private:
  FrameMap frames_;

  void recursiveFk(const JntArray& q, const Frame& par_frame, const SegmentMap::const_iterator& cur_it);
};

inline const Frame& TreeFkSolverPosAll::getFrame(const std::string& seg_name) const
{
  return frames_.at(seg_name);
}

inline const FrameMap& TreeFkSolverPosAll::getFrames() const
{
  return frames_;
}
}  // namespace kdl
