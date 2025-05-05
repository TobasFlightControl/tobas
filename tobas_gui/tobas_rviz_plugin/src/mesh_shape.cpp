#include "../include/tobas_rviz_plugin/mesh_shape.hpp"

#include <OgreMesh.h>
#include <OgreMeshManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreManualObject.h>
#include <rviz_common/logging.hpp>

namespace rviz_rendering
{
MeshShape::MeshShape(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
  : Shape(Shape::Mesh, scene_manager, parent_node), started_(false)
{
  static uint32_t count = 0;
  manual_object_ = scene_manager->createManualObject("MeshShape_ManualObject" + std::to_string(count++));
  material_->setCullingMode(Ogre::CULL_NONE);
}

MeshShape::~MeshShape()
{
  clear();
  scene_manager_->destroyManualObject(manual_object_);
}

void MeshShape::estimateVertexCount(size_t vcount)
{
  if (entity_ == nullptr && !started_) {
    manual_object_->estimateVertexCount(vcount);
  }
}

void MeshShape::beginTriangles()
{
  if (!started_ && entity_) {
    RVIZ_COMMON_LOG_WARNING("Cannot modify mesh once construction is complete");
    return;
  }

  if (!started_) {
    started_ = true;
    manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");
  }
}

void MeshShape::addVertex(const Ogre::Vector3& position)
{
  beginTriangles();
  manual_object_->position(position);
}

void MeshShape::addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal)
{
  beginTriangles();
  manual_object_->position(position);
  manual_object_->normal(normal);
}

void MeshShape::addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal, const Ogre::ColourValue& color)
{
  beginTriangles();
  manual_object_->position(position);
  manual_object_->normal(normal);
  manual_object_->colour(color);
}

void MeshShape::addNormal(const Ogre::Vector3& normal)
{
  manual_object_->normal(normal);
}

void MeshShape::addColor(const Ogre::ColourValue& color)
{
  manual_object_->colour(color);
}

void MeshShape::addTriangle(unsigned int v1, unsigned int v2, unsigned int v3)
{
  manual_object_->triangle(v1, v2, v3);
}

void MeshShape::endTriangles()
{
  if (started_) {
    started_ = false;
    manual_object_->end();
    static uint32_t count = 0;
    std::string name = "ConvertedMeshShape@" + std::to_string(count++);
    manual_object_->convertToMesh(name);
    entity_ = scene_manager_->createEntity(name);
    if (entity_) {
      entity_->setMaterialName(material_name_, "rviz_rendering");
      offset_node_->attachObject(entity_);
    }
    else {
      RVIZ_COMMON_LOG_ERROR("Unable to construct triangle mesh");
    }
  }
  else {
    RVIZ_COMMON_LOG_ERROR("No triangles added");
  }
}

void MeshShape::clear()
{
  if (entity_) {
    entity_->detachFromParent();
    const auto& mesh_name = entity_->getMesh()->getName();
    if (Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().getByName(mesh_name)) {
      Ogre::MeshManager::getSingleton().remove(mesh);
    }
    scene_manager_->destroyEntity(entity_);
    entity_ = nullptr;
  }
  manual_object_->clear();
  started_ = false;
}

}  // namespace rviz_rendering
