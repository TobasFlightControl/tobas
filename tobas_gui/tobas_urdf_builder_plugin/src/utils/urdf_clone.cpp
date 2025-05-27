#include "tobas_urdf_builder_plugin/utils/urdf_clone.hpp"

using namespace std;

namespace gui
{
namespace urdf_builder
{
namespace utils
{
urdf::GeometrySharedPtr clone(const urdf::GeometrySharedPtr& geometry)
{
  switch (geometry->type) {
    case urdf::Geometry::BOX: {
      const auto box = urdf::dynamic_pointer_cast<urdf::Box>(geometry);
      return make_shared<urdf::Box>(*box);
    }
    case urdf::Geometry::CYLINDER: {
      const auto cylinder = urdf::dynamic_pointer_cast<urdf::Cylinder>(geometry);
      return make_shared<urdf::Cylinder>(*cylinder);
    }
    case urdf::Geometry::SPHERE: {
      const auto sphere = urdf::dynamic_pointer_cast<urdf::Sphere>(geometry);
      return make_shared<urdf::Sphere>(*sphere);
    }
    case urdf::Geometry::MESH: {
      const auto mesh = urdf::dynamic_pointer_cast<urdf::Mesh>(geometry);
      return make_shared<urdf::Mesh>(*mesh);
    }
    default: {
      throw;
    }
  }
}

urdf::VisualSharedPtr clone(const urdf::VisualSharedPtr& visual)
{
  if (!visual) {
    return nullptr;
  }

  auto res = make_shared<urdf::Visual>(*visual);
  res->geometry = clone(visual->geometry);
  res->material = clone(visual->material);
  return res;
}

urdf::CollisionSharedPtr clone(const urdf::CollisionSharedPtr& collision)
{
  if (!collision) {
    return nullptr;
  }

  auto res = make_shared<urdf::Collision>(*collision);
  res->geometry = clone(collision->geometry);
  return res;
}

urdf::JointCalibrationSharedPtr clone(const urdf::JointCalibrationSharedPtr& calibration)
{
  if (!calibration) {
    return nullptr;
  }

  auto res = make_shared<urdf::JointCalibration>(*calibration);
  res->falling = clone(calibration->falling);
  res->rising = clone(calibration->rising);
  return res;
}

urdf::JointSharedPtr clone(const urdf::JointSharedPtr& joint)
{
  if (!joint) {
    return nullptr;
  }

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
  if (!link) {
    return nullptr;
  }

  auto res = make_shared<urdf::Link>(*link);
  res->inertial = clone(link->inertial);
  res->visual = clone(link->visual);
  res->collision = clone(link->collision);
  res->parent_joint = clone(link->parent_joint);

  res->collision_array.clear();
  for (const auto& collision : link->collision_array) {
    res->collision_array.push_back(clone(collision));
  }

  res->visual_array.clear();
  for (const auto& visual : link->visual_array) {
    res->visual_array.push_back(clone(visual));
  }

  res->child_links.clear();
  res->child_joints.clear();
  for (const auto& child_link : link->child_links) {
    const auto child_link_clone = clone(child_link);
    res->child_links.push_back(child_link_clone);
    res->child_joints.push_back(child_link_clone->parent_joint);
  }

  return res;
}
}  // namespace utils
}  // namespace urdf_builder
}  // namespace gui
