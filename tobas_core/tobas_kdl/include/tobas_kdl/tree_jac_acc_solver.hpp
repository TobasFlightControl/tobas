#pragma once

#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace kdl
{
/**
 * @brief xdd = J qd + Jd qd におけるJd qdの項を計算する．
 * qdd = 0, grav = 0としてRNEの順伝搬を行うことでJd qdが求められる．
 * cf. tree_id_solver_rne.cpp
 */
class TreeJacAccSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJacAccSolver(const Tree& tree);

  bool updateInternalDataStructures() override;

  int JntToCart(const JntArray& q, const JntArray& qd);

  inline const Accel& getJdqd(const std::string& seg_name) const;

private:
  RotationMap R_;
  TwistMap v_;
  AccelMap a_;
  AccelMap Jdqd_out_;

  void initialize();
  void JntToCartRec(const SegmentMap::const_iterator& segment, const JntArray& q, const JntArray& qd);
};

inline const Accel& TreeJacAccSolver::getJdqd(const std::string& seg_name) const
{
  return Jdqd_out_.at(seg_name);
}
}  // namespace kdl
