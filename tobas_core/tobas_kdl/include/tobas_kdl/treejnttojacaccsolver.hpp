#pragma once

#include "./treesolveri.hpp"
#include "./jntarray.hpp"

namespace tobas_kdl
{
/**
 * @brief xdd = J qd + Jd qd におけるJd qdの項を計算する．
 * qdd = 0, grav = 0としてRNEの順伝搬を行うことでJd qdが求められる．
 * cf. treeidsolver_rne.cpp
 */
class TreeJntToJacAccSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJntToJacAccSolver(const Tree& tree);

  void updateInternalDataStructures() override;

  int JntToCart(const JntArray& q, const JntArray& qd);

  const Accel& getJdqd(const std::string& seg_name) const
  {
    return Jdqd_out_.at(seg_name);
  }

private:
  RotationMap R_;
  TwistMap v_;
  AccelMap a_;
  AccelMap Jdqd_out_;

  void JntToCartRec(const SegmentMap::const_iterator& segment, const JntArray& q, const JntArray& qd);
};
}  // namespace tobas_kdl
