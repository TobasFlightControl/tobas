#pragma once

#include "./frames.hpp"
#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace kdl
{
/**
 * @brief This <strong>abstract</strong> class encapsulates the inverse
 * dynamics solver for a kdl::Tree.
 *
 */
class TreeIdSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeIdSolver(const Tree& tree) : super(tree)
  {
  }

  /**
   * Calculate inverse dynamics, from joint positions, velocity, acceleration, external forces
   * to joint torques/forces.
   *
   * @param q input joint positions
   * @param q_dot input joint velocities
   * @param q_dotdot input joint accelerations
   * @param f_ext the external forces (no gravity) on the segments
   *
   * @return if < 0 something went wrong
   */
  virtual int CartToJnt(const JntArray& q, const JntArray& q_dot, const JntArray& q_dotdot, const WrenchMap& f_ext) = 0;

  const JntArray& getEfforts() const
  {
    return effort_out_;
  }

protected:
  JntArray effort_out_;
};
}  // namespace kdl
