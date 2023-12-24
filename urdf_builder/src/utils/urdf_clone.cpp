#include "../../include/urdf_builder/utils/urdf_clone.hpp"

using namespace std;

namespace urdf_builder
{
namespace utils
{
urdf::VisualSharedPtr clone(const urdf::VisualSharedPtr& visual)
{
  if (!visual)
    return nullptr;

  auto res = make_shared<urdf::Visual>(*visual);
  res->geometry = clone(visual->geometry);
  res->material = clone(visual->material);
  return res;
}

urdf::CollisionSharedPtr clone(const urdf::CollisionSharedPtr& collision)
{
  if (!collision)
    return nullptr;

  auto res = make_shared<urdf::Collision>(*collision);
  res->geometry = clone(collision->geometry);
  return res;
}

urdf::JointCalibrationSharedPtr clone(const urdf::JointCalibrationSharedPtr& calibration)
{
  if (!calibration)
    return nullptr;

  auto res = make_shared<urdf::JointCalibration>(*calibration);
  res->falling = clone(calibration->falling);
  res->rising = clone(calibration->rising);
  return res;
}

urdf::JointSharedPtr clone(const urdf::JointSharedPtr& joint)
{
  if (!joint)
    return nullptr;

  auto res = make_shared<urdf::Joint>(*joint);
  res->dynamics = clone(joint->dynamics);
  res->limits = clone(joint->limits);
  res->safety = clone(joint->safety);
  res->calibration = clone(joint->calibration);
  res->mimic = clone(joint->mimic);
  return res;
}

urdf::LinkSharedPtr clone(const urdf::LinkSharedPtr& link)
{
  if (!link)
    return nullptr;

  auto res = make_shared<urdf::Link>(*link);
  res->inertial = clone(link->inertial);
  res->visual = clone(link->visual);
  res->collision = clone(link->collision);
  res->collision_array = clone(link->collision_array);
  res->visual_array = clone(link->visual_array);
  res->parent_joint = clone(link->parent_joint);
  res->child_joints = clone(link->child_joints);
  res->child_links = clone(link->child_links);
  return res;
}
}  // namespace utils
}  // namespace urdf_builder
