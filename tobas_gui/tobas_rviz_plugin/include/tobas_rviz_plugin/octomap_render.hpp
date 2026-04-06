// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>
#include <vector>

#include <OgrePrerequisites.h>
#include <octomap/octomap.h>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp>

namespace octomap
{
class OcTree;
}  // namespace octomap

namespace tobas
{
enum OctreeVoxelRenderMode
{
  kFree = 1,
  kOccupied = 2,
  kDisabled = 3
};

enum OctreeVoxelColorMode
{
  kZAxis,
  kProbability,
};

class OcTreeRender
{
public:
  OcTreeRender(
    const std::shared_ptr<const octomap::OcTree>& octree,
    OctreeVoxelRenderMode octree_voxel_rendering,
    OctreeVoxelColorMode octree_color_mode,
    size_t max_octree_depth,
    Ogre::SceneNode* parent_node);
  virtual ~OcTreeRender();

  void setPosition(const Ogre::Vector3& position);
  void setOrientation(const Ogre::Quaternion& orientation);

private:
  void setColor(double z_pos, double min_z, double max_z, double color_factor, rviz_rendering::PointCloud::Point* point);
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
  size_t octree_depth_;
};
}  // namespace tobas
