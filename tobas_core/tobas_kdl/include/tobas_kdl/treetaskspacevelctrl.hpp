#pragma once

#include "./treefksolverpos.hpp"
#include "./treeiksolvervel_pinv.hpp"
#include "./taskspacedamping.hpp"

namespace kdl
{
class TreeTaskSpaceVelCtrl : public TreeSolverI
{
  using super = TreeSolverI;

public:
  static constexpr double kDefaultTimeConst = 0.3;  // [s]

  explicit TreeTaskSpaceVelCtrl(const Tree& tree);

  void updateInternalDataStructures() override;

  int CartToJnt(const JntArray& cur_q, const FrameMap& tar_p);

  bool setLinearTimeConst(const Vector& t);
  bool setAngularTimeConst(const Vector& t);
  bool setLinearTimeConst(const double& t);
  bool setAngularTimeConst(const double& t);

  inline const JntArray& getVelocities() const;

private:
  TreeFkSolverPos fk_;
  TreeIkSolverVel_pinv ik_;

  TaskSpaceDamping gain_;
};

inline const JntArray& TreeTaskSpaceVelCtrl::getVelocities() const
{
  return ik_.getVelocities();
}
}  // namespace kdl
