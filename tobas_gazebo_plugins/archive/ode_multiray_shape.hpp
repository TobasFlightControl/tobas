#pragma once

#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/ode/common.h>
#include <gazebo/util/system.hh>

namespace gazebo
{
namespace physics
{
class GZ_PHYSICS_VISIBLE OdeMultiRayShape : public MultiRayShape
{
  static constexpr uint32_t kDefaultHorizontalSamples = 1;
  static constexpr uint32_t kDefaultVerticalSamples = 1;

  using super = MultiRayShape;

public:
  explicit OdeMultiRayShape(CollisionPtr parent);
  ~OdeMultiRayShape();

  void Init() override;
  void UpdateRays() override;
  void AddRay(const ignition::math::Vector3d& start, const ignition::math::Vector3d& end) override;

  std::vector<RayShapePtr>& rayShapes();

private:
  // SDF parameters
  double min_range_;
  double max_range_;
  double hor_min_angle_;
  double hor_max_angle_;
  uint32_t hor_samples_;
  double ver_min_angle_ = 0.;
  double ver_max_angle_ = 0.;
  uint32_t ver_samples_;

  double y_diff_;
  double p_diff_;

  dSpaceID super_space_id_;
  dSpaceID ray_space_id_;

  void getSdfParams();

  /* Ray-intersection callback. */
  static void updateCallback(void* data, dGeomID o1, dGeomID o2);
};
}  // namespace physics
}  // namespace gazebo
