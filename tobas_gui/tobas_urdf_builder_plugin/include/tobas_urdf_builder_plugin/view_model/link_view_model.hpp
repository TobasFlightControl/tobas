#pragma once

#include <memory>

#include "../utils/urdf_clone.hpp"
#include "./collision_view_model.hpp"
#include "./geometry_view_model.hpp"
#include "./inertial_view_model.hpp"
#include "./joint_view_model.hpp"
#include "./material_view_model.hpp"
#include "./visual_view_model.hpp"

namespace gui
{
namespace urdf_builder
{
namespace view_model
{
class LinkViewModel;

using LinkViewModelPtr = std::shared_ptr<LinkViewModel>;
using V_LinkViewModelPtr = std::vector<LinkViewModelPtr>;

class LinkViewModel : public BaseViewModel<urdf::Link, LinkViewModel>
{
public:
  explicit LinkViewModel(const urdf::LinkSharedPtr& model = nullptr);

  void sync() override;

  QString name() const;
  void name(const QString& name);

  const InertialViewModelPtr& inertial();
  const V_VisualViewModelPtr& visuals();
  const V_CollisionViewModelPtr& collisions();
  const JointViewModelPtr& joint();
  V_LinkViewModelPtr children() const;

  void add(const VisualViewModelPtr& visual);
  void remove(const VisualViewModelPtr& visual);

  void add(const CollisionViewModelPtr& collision);
  void remove(const CollisionViewModelPtr& collision);

private:
  InertialViewModelPtr inertial_;
  JointViewModelPtr joint_;
  V_VisualViewModelPtr visuals_;
  V_CollisionViewModelPtr collisions_;
};
}  // namespace view_model
}  // namespace urdf_builder
}  // namespace gui
