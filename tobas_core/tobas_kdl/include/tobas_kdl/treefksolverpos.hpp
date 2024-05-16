#pragma once

#include "./treesolveri.hpp"
#include "./frames.hpp"
#include "./jntarray.hpp"

namespace tobas_kdl
{
class TreeFkSolverPos : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeFkSolverPos(const Tree& tree);

  void updateInternalDataStructures() override;

  int JntToCart(const JntArray& q_in, const std::string& seg_name);

  const Frame& getFrame() const
  {
    return p_out_;
  }

private:
  Frame p_out_;

  Frame recursiveFk(const JntArray& q_in, const SegmentMap::const_iterator& it);
};
}  // namespace tobas_kdl
