#pragma once

#include <memory>
#include <Eigen/Geometry>
#include <OgreColourValue.h>
#include <geometric_shapes/shapes.h>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/objects/shape.hpp>

#include "./octomap_render.hpp"
#include "./class_forward.hpp"

namespace tobas
{
TOBAS_CLASS_FORWARD(OcTreeRender);  // Defines OcTreeRenderPtr, ConstPtr, WeakPtr... etc
TOBAS_CLASS_FORWARD(RenderShapes);  // Defines RenderShapesPtr, ConstPtr, WeakPtr... etc

class RenderShapes
{
public:
  RenderShapes(rviz_common::DisplayContext* context);
  ~RenderShapes();

  void renderShape(
    Ogre::SceneNode* node,
    const shapes::Shape* s,
    const Eigen::Isometry3d& p,
    OctreeVoxelRenderMode octree_voxel_rendering,
    OctreeVoxelColorMode octree_color_mode,
    const Ogre::ColourValue& color,
    double alpha);
  void updateShapeColors(double r, double g, double b, double a);
  void clear();

private:
  rviz_common::DisplayContext* context_;

  std::vector<std::unique_ptr<rviz_rendering::Shape>> scene_shapes_;
  std::vector<OcTreeRenderPtr> octree_voxel_grids_;
};
}  // namespace tobas
