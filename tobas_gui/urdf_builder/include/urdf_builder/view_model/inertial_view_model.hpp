#pragma once

#include <memory>

#include "./base_view_model.hpp"

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

  void buildInertiaSphere(double radius);
  void buildInertiaCylinder(double radius, double length);
  void buildInertiaBox(double x, double y, double z);
};

using InertialViewModelPtr = std::shared_ptr<InertialViewModel>;
}  // namespace view_model
}  // namespace urdf_builder
