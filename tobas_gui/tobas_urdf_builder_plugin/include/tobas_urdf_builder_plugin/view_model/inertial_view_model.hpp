#pragma once

#include <memory>

#include "./base_view_model.hpp"

namespace gui
{
namespace urdf_builder
{
namespace view_model
{
struct Inertia
{
  double ixx, ixy, ixz;
  double iyy, iyz;
  double izz;
};

class InertialViewModel : public BaseViewModel<urdf::Inertial, InertialViewModel>
{
public:
  using BaseViewModel<urdf::Inertial, InertialViewModel>::BaseViewModel;

  void sync() override;

  const urdf::Pose& origin() const;
  void origin(const urdf::Pose& origin);

  double mass() const;
  void mass(double mass);

  Inertia inertia() const;
  void inertia(const Inertia& inertia);

  /* Inertia wrt. CoM of a box. */
  void buildInertiaBox(double x, double y, double z);

  /* Inertia wrt. CoM of a cylinder. */
  void buildInertiaCylinder(double radius, double length);

  /* Inertia wrt. CoM of a sphere. */
  void buildInertiaSphere(double radius);
};

using InertialViewModelPtr = std::shared_ptr<InertialViewModel>;
}  // namespace view_model
}  // namespace urdf_builder
}  // namespace gui
