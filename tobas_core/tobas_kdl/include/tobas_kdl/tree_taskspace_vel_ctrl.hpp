#pragma once

#include "./taskspace_damping.hpp"
#include "./tree_fk_solver_pos.hpp"
#include "./tree_ik_solver_vel_pinv.hpp"

namespace kdl
{
class TreeTaskSpaceVelCtrl : public TreeSolverI
{
  using super = TreeSolverI;

public:
  static constexpr double kDefaultTimeConst = 0.3;  // [s]

  explicit TreeTaskSpaceVelCtrl(const Tree& tree);

  bool updateInternalDataStructures() override;

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
