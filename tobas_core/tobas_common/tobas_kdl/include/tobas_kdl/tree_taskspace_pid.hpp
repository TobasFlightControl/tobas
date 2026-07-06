// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./taskspace_damping.hpp"
#include "./taskspace_stiffness.hpp"
#include "./tree_fk_solver_vel.hpp"
#include "./tree_id_solver_rne.hpp"
#include "./tree_ik_solver_acc_rac.hpp"

namespace tobas
{
namespace kdl
{
class TreeTaskSpacePID : public TreeSolverI
{
  using super = TreeSolverI;

public:
  static constexpr double kDefaultStiffness = 25.0;
  static constexpr double kDefaultDamping = 10.0;

  explicit TreeTaskSpacePID(const Tree& tree, const Vector& grav = Vector(0, 0, -st::kGravity));

  bool updateInternalDataStructures() override;

  int cartToJnt(
    const JntArray& cur_q,
    const JntArray& cur_qd,
    const FrameMap& tar_p,
    const TwistMap& tar_v,
    const AccelMap& a_ff,
    const WrenchMap& f_ext = WrenchMap());

  bool setLinearStiffness(const Vector& kp);
  bool setAngularStiffness(const Vector& kp);
  bool setLinearDamping(const Vector& kd);
  bool setAngularDamping(const Vector& kd);
  bool setLinearStiffness(const double& kp);
  bool setAngularStiffness(const double& kp);
  bool setLinearDamping(const double& kd);
  bool setAngularDamping(const double& kd);

  inline const JntArray& getEfforts() const;

private:
  TreeFkSolverVel fk_;
  TreeIkSolverAcc_RAC rac_;
  TreeIdSolver_RNE rne_;

  TaskSpaceStiffness kp_;
  TaskSpaceDamping kd_;
};

inline const JntArray& TreeTaskSpacePID::getEfforts() const
{
  return rne_.getEfforts();
}
}  // namespace kdl
}  // namespace tobas
