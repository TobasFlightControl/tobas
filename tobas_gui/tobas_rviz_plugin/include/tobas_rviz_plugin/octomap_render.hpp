#pragma once

#include <vector>
#include <memory>
#include <octomap/octomap.h>
#include <OgrePrerequisites.h>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp>

namespace octomap
{
class OcTree;
}

namespace tobas
{
enum OctreeVoxelRenderMode
{
  OCTOMAP_FREE_VOXELS = 1,
  OCTOMAP_OCCUPIED_VOXELS = 2,
  OCTOMAP_DISABLED = 3
};

enum OctreeVoxelColorMode
{
  OCTOMAP_Z_AXIS_COLOR,
  OCTOMAP_PROBABLILTY_COLOR,
};

class OcTreeRender
{
public:
  OcTreeRender(
    const std::shared_ptr<const octomap::OcTree>& octree,
    OctreeVoxelRenderMode octree_voxel_rendering,
    OctreeVoxelColorMode octree_color_mode,
    std::size_t max_octree_depth,
    Ogre::SceneNode* parent_node);
  virtual ~OcTreeRender();

  void setPosition(const Ogre::Vector3& position);
  void setOrientation(const Ogre::Quaternion& orientation);

private:
  void
  setColor(double z_pos, double min_z, double max_z, double color_factor, rviz_rendering::PointCloud::Point* point);
  void setProbColor(double prob, rviz_rendering::PointCloud::Point* point);

  void octreeDecoding(
    const std::shared_ptr<const octomap::OcTree>& octree,
    OctreeVoxelRenderMode octree_voxel_rendering,
    OctreeVoxelColorMode octree_color_mode);

  // Ogre-rviz point clouds
  std::vector<rviz_rendering::PointCloud*> cloud_;
  std::shared_ptr<const octomap::OcTree> octree_;

  Ogre::SceneNode* scene_node_;

  double colorFactor_;
  std::size_t octree_depth_;
};
}  // namespace tobas
