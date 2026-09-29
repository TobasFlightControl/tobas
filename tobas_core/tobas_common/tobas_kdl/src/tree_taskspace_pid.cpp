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
constexpr double kDefaultStiffness = 25.0;
constexpr double kDefaultDamping = 10.0;
}  // namespace

TreeTaskSpacePID::TreeTaskSpacePID(const Tree& tree, const Vector& grav)
  : super(tree)
  , fk_(tree)
  , rac_(tree)
  , rne_(tree, grav)
  , kp_(Vector::Constant(kDefaultStiffness), Vector::Constant(kDefaultStiffness))
  , kd_(Vector::Constant(kDefaultDamping), Vector::Constant(kDefaultDamping))
{
}

void TreeTaskSpacePID::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  fk_.updateInternalDataStructures();
  rac_.updateInternalDataStructures();
  rne_.updateInternalDataStructures();
}

int TreeTaskSpacePID::cartToJnt(
  const JntArray& cur_q,
  const JntArray& cur_qd,
  const FrameMap& tar_p,
  const TwistMap& tar_v,
  const AccelMap& a_ff,
  const WrenchMap& f_ext)
{
  if (!isUpToDate()) {
    return setDefaultError(kNotUpToDate);
  }
  if (cur_q.rows() != nj_ || cur_qd.rows() != nj_) {
    setDefaultError(kSizeMismatch);
  }
  if (tar_p.size() != tar_v.size() || tar_p.size() != a_ff.size()) {
    return setDefaultError(kSizeMismatch);
  }

  // Create target acceleration map.
  AccelMap tar_a;
  for (const auto& [tar_pi, tar_vi, ai_ff] : std::views::zip(tar_p, tar_v, a_ff)) {
    // Check if all keys match.
    const auto& seg_name = tar_pi.first;
    if (tar_vi.first != seg_name || ai_ff.first != seg_name) {
      error_msg_ = "The keys of input maps do not match.";
      return (error_code_ = kOutputRange);
    }

    // Compute current frame and twist.
    if (fk_.jntToCart(cur_q, cur_qd, seg_name) < 0) {
      return copyError(fk_);
    }
    const auto& cur_pv = fk_.getFrameVel();
    const auto cur_p = cur_pv.getFrame();
    const auto cur_v = cur_pv.getTwist();

    // Compute target cartesian acceleration.
    // TODO: Add the I term.
    tar_a[seg_name] = ai_ff.second + kp_ * (tar_pi.second - cur_p) + kd_ * (tar_vi.second - cur_v);
  }

  // Compute target joint accelerations.
  if (rac_.cartToJnt(cur_q, cur_qd, tar_a) < 0) {
    return copyError(rac_);
  }

  // Compute target joint efforts.
  if (rne_.cartToJnt(cur_q, cur_qd, rac_.getAccelerations(), f_ext) < 0) {
    return copyError(rne_);
  }

  return setDefaultError(kNoError);
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
