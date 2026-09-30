// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_taskspace_pid.hpp"

#include <cassert>
#include <ranges>

namespace tobas
{
namespace kdl
{
namespace
{

}  // namespace

TreeTaskSpacePID::TreeTaskSpacePID(const Tree& tree, const Vector& grav)
  : super(tree), fk_(tree), rac_(tree), rne_(tree, grav)
{
  constexpr double kDefaultStiffness = 25.0;
  setLinearStiffness(kDefaultStiffness);
  setAngularStiffness(kDefaultStiffness);

  constexpr double kDefaultDamping = 10.0;
  setLinearDamping(kDefaultDamping);
  setAngularDamping(kDefaultDamping);
}

void TreeTaskSpacePID::updateInternalDataStructures()
{
  fk_.updateInternalDataStructures();
  rac_.updateInternalDataStructures();
  rne_.updateInternalDataStructures();
}

std::expected<JntArray, std::string> TreeTaskSpacePID::cartToJnt(
  const JntArray& cur_q,
  const JntArray& cur_qd,
  const FrameMap& tar_p,
  const TwistMap& tar_v,
  const AccelMap& a_ff,
  const WrenchMap& f_ext)
{
  assert(cur_q.size() == tree_.getNrOfJoints());
  assert(cur_qd.size() == tree_.getNrOfJoints());
  assert(tar_p.size() == tar_v.size());
  assert(tar_p.size() == a_ff.size());

  // Create target acceleration map.
  AccelMap tar_a;
  for (const auto& [tar_pi, tar_vi, ai_ff] : std::views::zip(tar_p, tar_v, a_ff)) {
    // Check if all keys match.
    const auto& seg_name = tar_pi.first;
    assert(tar_vi.first == seg_name);
    assert(ai_ff.first == seg_name);

    // Compute current frame and twist.
    const auto fk_result = fk_.jntToCart(cur_q, cur_qd, seg_name);
    const auto& cur_pv = fk_result;
    const auto cur_p = cur_pv.getFrame();
    const auto cur_v = cur_pv.getTwist();

    // Compute target cartesian acceleration.
    // TODO: Add the I term.
    tar_a[seg_name] = ai_ff.second + kp_ * (tar_pi.second - cur_p) + kd_ * (tar_vi.second - cur_v);
  }

  // Compute target joint accelerations.
  const auto tar_qdd = rac_.cartToJnt(cur_q, cur_qd, tar_a);
  if (!tar_qdd) {
    return std::unexpected("Failed to solve RAC: " + tar_qdd.error());
  }

  // Compute target joint efforts.
  return rne_.cartToJnt(cur_q, cur_qd, tar_qdd.value(), f_ext);
}

void TreeTaskSpacePID::setLinearStiffness(const Vector& kp)
{
  assert(kp.x() >= 0.0 && kp.y() >= 0.0 && kp.z() >= 0.0);
  kp_.linear = kp;
}

void TreeTaskSpacePID::setAngularStiffness(const Vector& kp)
{
  assert(kp.x() >= 0.0 && kp.y() >= 0.0 && kp.z() >= 0.0);
  kp_.angular = kp;
}

void TreeTaskSpacePID::setLinearDamping(const Vector& kd)
{
  assert(kd.x() >= 0.0 && kd.y() >= 0.0 && kd.z() >= 0.0);
  kp_.linear = kd;
}

void TreeTaskSpacePID::setAngularDamping(const Vector& kd)
{
  assert(kd.x() >= 0.0 && kd.y() >= 0.0 && kd.z() >= 0.0);
  kp_.angular = kd;
}

void TreeTaskSpacePID::setLinearStiffness(const double& kp)
{
  assert(kp >= 0.0);
  kp_.linear.fill(kp);
}

void TreeTaskSpacePID::setAngularStiffness(const double& kp)
{
  assert(kp >= 0.0);
  kp_.angular.fill(kp);
}

void TreeTaskSpacePID::setLinearDamping(const double& kd)
{
  assert(kd >= 0.0);
  kd_.linear.fill(kd);
}

void TreeTaskSpacePID::setAngularDamping(const double& kd)
{
  assert(kd >= 0.0);
  kd_.angular.fill(kd);
}
}  // namespace kdl
}  // namespace tobas
