#pragma once

#include <vector>
#include <string>
#include <unordered_set>

#include "./tree_solver_i.hpp"
#include "./tree_fk_solver_pos.hpp"
#include "./tree_ik_solver_vel_pinv.hpp"
#include "./tree_joint_parser.hpp"

namespace kdl
{
/**
 * Implementation of a general inverse position kinematics algorithm to calculate the position
 * transformation from Cartesian to joint space of a general kdl::Tree. This class has been
 * derived from the TreeIkSolverPos_NR_JL class, but was modified for online solving for use in
 * realtime systems. Thus, the calculation is only done once, meaning that no iteration is done,
 * because this solver is intended to run at a high frequency. It enforces velocity limits in task
 * as well as in joint space. It also takes joint limits into account.
 */
class TreeIkSolverPos_Online : public TreeSolverI
{
  using super = TreeSolverI;

public:
  static constexpr double kDefaultMaxLinearVelocity = 1.;     // [m/s]
  static constexpr double kDefaultMaxAngularVelocity = M_PI;  // [rad/s]

  explicit TreeIkSolverPos_Online(const Tree& tree);

  void updateInternalDataStructures() override;

  int CartToJnt(const JntArray& q_in, const FrameMap& p_in, const double& dt);

  bool setMaxLinearVelocity(const double& max_linvel);
  bool setMaxAngularVelocity(const double& max_angvel);

  const JntArray& getPositions() const
  {
    return q_out_;
  }

private:
  double max_linvel_ = kDefaultMaxLinearVelocity;
  double max_angvel_ = kDefaultMaxAngularVelocity;

  TreeFkSolverPos fksolver_;
  TreeIkSolverVel_pinv iksolver_;
  TreeJointParser jntparser_;

  JntArray q_out_;

  /**
   * Scales the class member kdl::JntArray q_dot_, if one (or more) joint velocity exceeds the
   * maximum value. Scaling is done proportional to the biggest overshoot among all joint
   * velocities.
   */
  void enforceJointVelLimits(JntArray& qd, const double& dt);

  /**
   * Scales translational and rotational velocity vectors of the class member kdl::Twist
   * twist_, if at least one of both exceeds the maximum value/length. Scaling is done proportional
   * to the biggest overshoot among both velocities.
   */
  void enforceCartVelLimits(Twist& twist, const double& dt);
};
}  // namespace kdl
