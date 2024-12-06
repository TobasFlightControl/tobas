#include "../include/tobas_kdl/tree_ik_solver_pos_online.hpp"

using namespace std;

namespace kdl
{
TreeIkSolverPos_Online::TreeIkSolverPos_Online(const Tree& tree)
  : super(tree), fksolver_(tree), iksolver_(tree), jntparser_(tree)
{
}

void TreeIkSolverPos_Online::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  fksolver_.updateInternalDataStructures();
  iksolver_.updateInternalDataStructures();
  jntparser_.updateInternalDataStructures();
}

int TreeIkSolverPos_Online::CartToJnt(const JntArray& q_in, const FrameMap& p_in, const double& dt)
{
  if (!isUpToDate())
    return setDefaultError(E_NOT_UP_TO_DATE);
  if (q_in.rows() != nj_)
    return setDefaultError(E_SIZE_MISMATCH);
  if (dt < 0)
    return setDefaultError(E_NEGATIVE_DELTA_TIME);

  // Compute delta twists
  TwistMap delta_twists;
  for (const auto& [seg_name, frame] : p_in)
  {
    if (fksolver_.JntToCart(q_in, seg_name) < 0)
      return copyError(fksolver_);
    auto delta_t = (frame - fksolver_.getFrame()).toTwist();
    enforceCartVelLimits(delta_t, dt);
    delta_twists[seg_name] = delta_t;
  }

  // Compute delta q
  if (iksolver_.CartToJnt(q_in, delta_twists) < 0)
    return copyError(iksolver_);
  auto delta_q = iksolver_.getVelocities();
  enforceJointVelLimits(delta_q, dt);

  // Integrate
  q_out_ = q_in + delta_q;

  // Limit joint positions
  const auto& lb = jntparser_.lowerLimits().data;
  const auto& ub = jntparser_.upperLimits().data;
  q_out_.data = q_out_.data.cwiseMax(lb).cwiseMin(ub);

  return setDefaultError(E_NOERROR);
}

bool TreeIkSolverPos_Online::setMaxLinearVelocity(const double& max_linvel)
{
  if (max_linvel < 0)
    return false;

  max_linvel_ = max_linvel;
  return true;
}

bool TreeIkSolverPos_Online::setMaxAngularVelocity(const double& max_angvel)
{
  if (max_angvel < 0)
    return false;

  max_angvel_ = max_angvel;
  return true;
}

void TreeIkSolverPos_Online::enforceJointVelLimits(JntArray& delta_q, const double& dt)
{
  // check, if one (or more) joint velocities exceed the maximum value
  // and if so, safe the biggest overshoot for scaling delta_q properly
  // to keep the direction of the velocity vector the same
  double rel_os_max = 0.;  // the biggest relative overshoot
  bool max_exceeded = false;

  for (size_t i = 0; i < nj_; ++i)
  {
    const auto delta_q_max = jntparser_.maxVelocity(i) * dt;
    if (delta_q(i) > delta_q_max)
    {
      max_exceeded = true;
      const auto rel_os = (delta_q(i) - delta_q_max) / delta_q_max;
      if (rel_os > rel_os_max)
        rel_os_max = rel_os;
    }
    else if (delta_q(i) < -delta_q_max)
    {
      max_exceeded = true;
      const auto rel_os = (-delta_q(i) - delta_q_max) / delta_q_max;
      if (rel_os > rel_os_max)
        rel_os_max = rel_os;
    }
  }

  // scales delta_q, if one joint exceeds the maximum value
  if (max_exceeded == true)
    delta_q *= 1 / (1 + rel_os_max);
}

void TreeIkSolverPos_Online::enforceCartVelLimits(Twist& delta_t, const double& dt)
{
  const auto delta_lin_norm = delta_t.vel.norm();
  const auto delta_ang_norm = delta_t.rot.norm();
  const auto delta_lin_max = max_linvel_ * dt;
  const auto delta_ang_max = max_angvel_ * dt;

  if (delta_lin_norm > delta_lin_max || delta_ang_norm > delta_ang_max)
  {
    if (delta_lin_norm > delta_ang_norm)
    {
      delta_t.vel = delta_t.vel * (delta_lin_max / delta_lin_norm);
      delta_t.rot = delta_t.rot * (delta_lin_max / delta_lin_norm);
    }
    else if (delta_ang_norm > delta_lin_norm)
    {
      delta_t.vel = delta_t.vel * (delta_ang_max / delta_ang_norm);
      delta_t.rot = delta_t.rot * (delta_ang_max / delta_ang_norm);
    }
  }
}
}  // namespace kdl
