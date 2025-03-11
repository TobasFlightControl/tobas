#include "../../include/tobas_urdf_builder_plugin/ogre_helpers/static_link_updater.hpp"

inline static Ogre::Vector3 URDFVector3ToOgre(const urdf::Vector3& v)
{
  return Ogre::Vector3(v.x, v.y, v.z);
}

inline static Ogre::Quaternion URDFRotationToOgre(const urdf::Rotation& r)
{
  return Ogre::Quaternion(r.w, r.x, r.y, r.z);
}

namespace gui
{
namespace urdf_builder
{
namespace ogre
{
StaticLinkUpdater::StaticLinkUpdater(urdf::ModelSharedPtr urdf) : urdf_(std::move(urdf))
{
  for (const auto& link : urdf_->links_)
    transforms_[link.first] = findTransform(link.second);
}

bool StaticLinkUpdater::getLinkTransforms(
  const std::string& link_name,
  Ogre::Vector3& visual_position,
  Ogre::Quaternion& visual_orientation,
  Ogre::Vector3& collision_position,
  Ogre::Quaternion& collision_orientation) const
{
  const auto link = urdf_->getLink(link_name);
  if (!link)
  {
    setLinkStatus(rviz_common::properties::StatusProperty::Error, link_name, "Transform not found");
    return false;
  }

  const auto& vt = transforms_.at(link->name);
  visual_position = vt.getTrans();
  visual_orientation = vt.linear();
  collision_position = visual_position;
  collision_orientation = visual_orientation;

  setLinkStatus(rviz_common::properties::StatusProperty::Ok, link_name, "Transform OK");
  return true;
}

void StaticLinkUpdater::setLinkStatus(rviz_common::properties::StatusLevel, const std::string&, const std::string&)
  const
{
}

Ogre::Matrix4 StaticLinkUpdater::findTransform(const urdf::LinkConstSharedPtr& link)
{
  std::vector<Ogre::Matrix4> matrices;

  auto cur = link;
  while (cur && urdf_->getRoot() != cur)
  {
    if (!cur->parent_joint)
      break;
    const auto& pose = cur->parent_joint->parent_to_joint_origin_transform;
    Ogre::Matrix4 m;
    m.makeTransform(URDFVector3ToOgre(pose.position), Ogre::Vector3(1, 1, 1), URDFRotationToOgre(pose.rotation));

    matrices.push_back(m);
    cur = urdf_->getLink(cur->parent_joint->parent_link_name);
  }

  std::reverse(matrices.begin(), matrices.end());

  matrices.push_back(Ogre::Matrix4::IDENTITY);
  auto result = matrices[0];
  for (size_t i = 1; i < matrices.size(); ++i)
    result = result * matrices[i];
  return result;
}
}  // namespace ogre
}  // namespace urdf_builder
}  // namespace gui
