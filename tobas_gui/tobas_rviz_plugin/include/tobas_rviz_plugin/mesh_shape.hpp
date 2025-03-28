#pragma once

#include <rviz_rendering/objects/shape.hpp>

namespace Ogre
{
class ManualObject;
}

namespace rviz_rendering
{
/* This class allows constructing Ogre shapes manually, from triangle lists. */
class MeshShape : public Shape
{
public:
  /**
   * @brief Constructor
   *
   * @param scene_manager The scene manager this object is associated with
   * @param parent_node A scene node to use as the parent of this object.  If nullptr, uses the root scene node.
   */
  MeshShape(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = nullptr);
  ~MeshShape() override;

  /* Estimate the number of vertices ahead of time. */
  void estimateVertexCount(size_t vcount);

  /* Start adding triangles to the mesh */
  void beginTriangles();

  /**
   * @brief Add a vertex to the mesh (no normal defined). If using
   * this function only (not using addTriangle()) it is assumed that
   * triangles are added by specifying the 3 vertices in order (3
   * consecutive calls to this function). This means there must be
   * 3*n calls to this function to add n triangles. If addTriangle()
   * is used, indexing in the defined vertices is done.
   */
  void addVertex(const Ogre::Vector3& position);

  /**
   * @brief Add a vertex to the mesh with a normal defined. If using
   * this function only (not using addTriangle()) it is assumed that
   * triangles are added by specifying the 3 vertices in order (3
   * consecutive calls to this function). This means there must be
   * 3*n calls to this function to add n triangles.If addTriangle()
   * is used, indexing in the defined vertices is done.
   */
  void addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal);

  /**
   * @brief Add a vertex to the mesh with normal and color defined. If using
   * this function only (not using addTriangle()) it is assumed that
   * triangles are added by specifying the 3 vertices in order (3
   * consecutive calls to this function). This means there must be
   * 3*n calls to this function to add n triangles.If addTriangle()
   * is used, indexing in the defined vertices is done.
   */
  void addVertex(const Ogre::Vector3& position, const Ogre::Vector3& normal, const Ogre::ColourValue& color);

  /* Add normal for a vertex */
  void addNormal(const Ogre::Vector3& normal);

  /* Add color for a vertex */
  void addColor(const Ogre::ColourValue& color);

  /* Add a triangle by indexing in the defined vertices. */
  void addTriangle(unsigned int p1, unsigned int p2, unsigned int p3);

  /**
   * @brief Notify that the set of triangles to add is complete.
   * No more triangles can be added, beginTriangles() can no longer be called unless clear() was called.
   */
  void endTriangles();

  /* Clear the mesh */
  void clear();

  /* Get the manual object created for the mesh */
  Ogre::ManualObject* getManualObject()
  {
    return manual_object_;
  }

private:
  bool started_;  // true in between calls to beginTriangles() and endTriangles()
  Ogre::ManualObject* manual_object_;
};

}  // namespace rviz_rendering
