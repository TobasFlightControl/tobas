// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_taskspace_pid.hpp"

#include <cassert>
#include <ranges>

namespace tobas::kdl
{
namespace
{

}  // namespace

TreeTaskSpacePID::TreeTaskSpacePID(const Tree& _tree, const Vector& _grav)
  : super(_tree), fk_(_tree), rac_(_tree), rne_(_tree, _grav)
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
  const JntArray& _cur_q,
  const JntArray& _cur_qd,
  const FrameMap& _tar_p,
  const TwistMap& _tar_v,
  const AccelMap& _a_ff,
  const WrenchMap& _f_ext)
{
  assert(_cur_q.size() == tree_.getNrOfJoints());
  assert(_cur_qd.size() == tree_.getNrOfJoints());
  assert(_tar_p.size() == _tar_v.size());
  assert(_tar_p.size() == _a_ff.size());

  // Create target acceleration map.
  AccelMap tar_a;
  for (const auto& [tar_pi, tar_vi, ai_ff] : std::views::zip(_tar_p, _tar_v, _a_ff)) {
    // Check if all keys match.
    const auto& seg_name = tar_pi.first;
    assert(tar_vi.first == seg_name);
    assert(ai_ff.first == seg_name);

    // Compute current frame and twist.
    const auto fk_result = fk_.jntToCart(_cur_q, _cur_qd, seg_name);
    const auto& cur_pv = fk_result;
    const auto cur_p = cur_pv.getFrame();
    const auto cur_v = cur_pv.getTwist();

    // Compute target cartesian acceleration.
    // TODO: Add the I term.
    tar_a[seg_name] = ai_ff.second + kp_ * (tar_pi.second - cur_p) + kd_ * (tar_vi.second - cur_v);
  }

  // Compute target joint accelerations.
  const auto tar_qdd = rac_.cartToJnt(_cur_q, _cur_qd, tar_a);
  if (!tar_qdd) {
    return std::unexpected("Failed to solve RAC: " + tar_qdd.error());
  }

  // Compute target joint efforts.
  return rne_.cartToJnt(_cur_q, _cur_qd, tar_qdd.value(), _f_ext);
}

void TreeTaskSpacePID::setLinearStiffness(const Vector& _kp)
{
  assert(_kp.x() >= 0.0 && _kp.y() >= 0.0 && _kp.z() >= 0.0);
  kp_.linear = _kp;
}

void TreeTaskSpacePID::setAngularStiffness(const Vector& _kp)
{
  assert(_kp.x() >= 0.0 && _kp.y() >= 0.0 && _kp.z() >= 0.0);
  kp_.angular = _kp;
}

void TreeTaskSpacePID::setLinearDamping(const Vector& _kd)
{
  assert(_kd.x() >= 0.0 && _kd.y() >= 0.0 && _kd.z() >= 0.0);
  kp_.linear = _kd;
}

void TreeTaskSpacePID::setAngularDamping(const Vector& _kd)
{
  assert(_kd.x() >= 0.0 && _kd.y() >= 0.0 && _kd.z() >= 0.0);
  kp_.angular = _kd;
}

void TreeTaskSpacePID::setLinearStiffness(double _kp)
{
  assert(_kp >= 0.0);
  kp_.linear.fill(_kp);
}

void TreeTaskSpacePID::setAngularStiffness(double _kp)
{
  assert(_kp >= 0.0);
  kp_.angular.fill(_kp);
}

void TreeTaskSpacePID::setLinearDamping(double _kd)
{
  assert(_kd >= 0.0);
  kd_.linear.fill(_kd);
}

void TreeTaskSpacePID::setAngularDamping(double _kd)
{
  assert(_kd >= 0.0);
  kd_.angular.fill(_kd);
}
}  // namespace tobas::kdl
