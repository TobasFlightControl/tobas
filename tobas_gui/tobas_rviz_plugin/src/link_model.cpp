#include <geometric_shapes/check_isometry.h>
#include <geometric_shapes/shape_operations.h>

#include "../include/tobas_rviz_plugin/link_model.hpp"
#include "../include/tobas_rviz_plugin/joint_model.hpp"
#include "../include/tobas_rviz_plugin/aabb.hpp"

namespace tobas
{
LinkModel::LinkModel(const std::string& name, size_t link_index)
  : name_(name),
    link_index_(link_index),
    parent_joint_model_(nullptr),
    parent_link_model_(nullptr),
    is_parent_joint_fixed_(false),
    joint_origin_transform_is_identity_(true),
    first_collision_body_transform_index_(-1)
{
  joint_origin_transform_.setIdentity();
}

LinkModel::~LinkModel() = default;

void LinkModel::setJointOriginTransform(const Eigen::Isometry3d& transform)
{
  ASSERT_ISOMETRY(transform)  // unsanitized input, could contain a non-isometry
  joint_origin_transform_ = transform;
  joint_origin_transform_is_identity_ =
    joint_origin_transform_.linear().isIdentity()
    && joint_origin_transform_.translation().norm() < std::numeric_limits<double>::epsilon();
}

void LinkModel::setParentJointModel(const JointModel* joint)
{
  parent_joint_model_ = joint;
  is_parent_joint_fixed_ = joint->getType() == JointModel::FIXED;
}

void LinkModel::setGeometry(const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Isometry3d& origins)
{
  shapes_ = shapes;
  collision_origin_transform_ = origins;
  collision_origin_transform_is_identity_.resize(collision_origin_transform_.size());

  AABB aabb;

  for (std::size_t i = 0; i < shapes_.size(); ++i) {
    ASSERT_ISOMETRY(collision_origin_transform_[i])  // unsanitized input, could contain a non-isometry
    collision_origin_transform_is_identity_[i] =
      (collision_origin_transform_[i].linear().isIdentity()
       && collision_origin_transform_[i].translation().norm() < std::numeric_limits<double>::epsilon()) ?
        1 :
        0;
    Eigen::Isometry3d transform = collision_origin_transform_[i];

    if (shapes_[i]->type != shapes::MESH) {
      Eigen::Vector3d extents = shapes::computeShapeExtents(shapes_[i].get());
      aabb.extendWithTransformedBox(transform, extents);
    }
    else {
      // we cannot use shapes::computeShapeExtents() for meshes, since that method does not provide information about
      // the offset of the mesh origin
      const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(shapes_[i].get());
      for (unsigned int j = 0; j < mesh->vertex_count; ++j) {
        aabb.extend(transform * Eigen::Map<Eigen::Vector3d>(&mesh->vertices[3 * j]));
      }
    }
  }

  centered_bounding_box_offset_ = aabb.center();
  if (shapes_.empty()) {
    shape_extents_.setZero();
  }
  else {
    shape_extents_ = aabb.sizes();
  }
}

void LinkModel::setVisualMesh(
  const std::string& visual_mesh,
  const Eigen::Isometry3d& origin,
  const Eigen::Vector3d& scale)
{
  visual_mesh_filename_ = visual_mesh;
  visual_mesh_origin_ = origin;
  visual_mesh_scale_ = scale;
}
}  // namespace tobas
