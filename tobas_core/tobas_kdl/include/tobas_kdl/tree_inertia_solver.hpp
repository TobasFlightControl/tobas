#pragma once

#include "./jntarray.hpp"
#include "./rigid_body_inertia.hpp"
#include "./tree_solver_i.hpp"

namespace kdl
{
class TreeInertiaSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeInertiaSolver(const Tree& tree);

  bool updateInternalDataStructures() override;

  /* ルートリンク周りの質量特性を計算する． */
  int JntToCart(const JntArray& q);

  inline const RigidBodyInertia& getInertia() const
  {
    return I_.at(tree_.getRootSegment()->first);
  }

private:
  std::map<std::string, Frame> X_;
  std::map<std::string, RigidBodyInertia> I_;

  void initialize();
  void step(const SegmentMap::const_iterator& cur_it, const JntArray& q);
};
}  // namespace kdl
