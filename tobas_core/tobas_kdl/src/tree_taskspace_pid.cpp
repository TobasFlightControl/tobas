#include <ranges>

#include "../include/tobas_kdl/tree_taskspace_pid.hpp"

using namespace std;

namespace kdl
{
TreeTaskSpacePID::TreeTaskSpacePID(const Tree& tree, const Vector& grav)
  : super(tree),
    fk_(tree),
    rac_(tree),
    rne_(tree, grav),
    kp_(Vector::Constant(kDefaultStiffness), Vector::Constant(kDefaultStiffness)),
    kd_(Vector::Constant(kDefaultDamping), Vector::Constant(kDefaultDamping))
{
}

bool TreeTaskSpacePID::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  if (!fk_.updateInternalDataStructures()) {
    return false;
  }
  if (!rac_.updateInternalDataStructures()) {
    return false;
  }
  if (!rne_.updateInternalDataStructures()) {
    return false;
  }

  return true;
}

int TreeTaskSpacePID::CartToJnt(
  const JntArray& cur_q,
  const JntArray& cur_qd,
  const FrameMap& tar_p,
  const TwistMap& tar_v,
  const AccelMap& a_ff,
  const WrenchMap& f_ext)
{
  if (!isUpToDate()) {
    return setDefaultError(E_NOT_UP_TO_DATE);
  }
  if (cur_q.rows() != nj_ || cur_qd.rows() != nj_) {
    setDefaultError(E_SIZE_MISMATCH);
  }
  if (tar_p.size() != tar_v.size() || tar_p.size() != a_ff.size()) {
    return setDefaultError(E_SIZE_MISMATCH);
  }

  // Create target acceleration map
  AccelMap tar_a;
  for (const auto& [tar_pi, tar_vi, ai_ff] : views::zip(tar_p, tar_v, a_ff)) {
    // Check if all keys match
    const auto& seg_name = tar_pi.first;
    if (tar_vi.first != seg_name || ai_ff.first != seg_name) {
      error_msg_ = "The keys of input maps do not match.";
      return (error_code_ = E_OUT_OF_RANGE);
    }

    // Compute current frame and twist
    if (fk_.JntToCart(cur_q, cur_qd, seg_name) < 0) {
      return copyError(fk_);
    }
    const auto& cur_pv = fk_.getFrameVel();
    const auto cur_p = cur_pv.getFrame();
    const auto cur_v = cur_pv.getTwist();

    // Compute target cartesian acceleration
    // TODO: I要素を加える
    tar_a[seg_name] = ai_ff.second + kp_ * (tar_pi.second - cur_p) + kd_ * (tar_vi.second - cur_v);
  }

  // Compute target joint accelerations
  if (rac_.CartToJnt(cur_q, cur_qd, tar_a) < 0) {
    return copyError(rac_);
  }

  // Compute target joint efforts
  if (rne_.CartToJnt(cur_q, cur_qd, rac_.getAccelerations(), f_ext) < 0) {
    return copyError(rne_);
  }

  return setDefaultError(E_NOERROR);
}

bool TreeTaskSpacePID::setLinearStiffness(const Vector& kp)
{
  if (kp.x() < 0 || kp.y() < 0 || kp.z() < 0) {
    return false;
  }

  kp_.linear = kp;
  return true;
}

bool TreeTaskSpacePID::setAngularStiffness(const Vector& kp)
{
  if (kp.x() < 0 || kp.y() < 0 || kp.z() < 0) {
    return false;
  }

  kp_.angular = kp;
  return true;
}

bool TreeTaskSpacePID::setLinearDamping(const Vector& kd)
{
  if (kd.x() < 0 || kd.y() < 0 || kd.z() < 0) {
    return false;
  }

  kp_.linear = kd;
  return true;
}

bool TreeTaskSpacePID::setAngularDamping(const Vector& kd)
{
  if (kd.x() < 0 || kd.y() < 0 || kd.z() < 0) {
    return false;
  }

  kp_.angular = kd;
  return true;
}

bool TreeTaskSpacePID::setLinearStiffness(const double& kp)
{
  if (kp < 0) {
    return false;
  }

  kp_.linear.fill(kp);
  return true;
}

bool TreeTaskSpacePID::setAngularStiffness(const double& kp)
{
  if (kp < 0) {
    return false;
  }

  kp_.angular.fill(kp);
  return true;
}

bool TreeTaskSpacePID::setLinearDamping(const double& kd)
{
  if (kd < 0) {
    return false;
  }

  kd_.linear.fill(kd);
  return true;
}

bool TreeTaskSpacePID::setAngularDamping(const double& kd)
{
  if (kd < 0) {
    return false;
  }

  kd_.angular.fill(kd);
  return true;
}
}  // namespace kdl
