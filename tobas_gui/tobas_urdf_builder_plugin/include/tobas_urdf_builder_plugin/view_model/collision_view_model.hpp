#pragma once

#include "./base_view_model.hpp"
#include "./geometry_view_model.hpp"

namespace gui
{
namespace urdf_builder
{
namespace view_model
{
class CollisionViewModel : public BaseViewModel<urdf::Collision, CollisionViewModel>
{
public:
  explicit CollisionViewModel(const urdf::CollisionSharedPtr& model);

  void sync() override;

  const urdf::Pose& origin() const;
  void origin(const urdf::Pose& origin);

  QString name() const;
  void name(const QString& name);

  const GeometryViewModelPtr& geometry();

private:
  GeometryViewModelPtr geometry_vm_;
};

using CollisionViewModelPtr = std::shared_ptr<CollisionViewModel>;
using V_CollisionViewModelPtr = std::vector<CollisionViewModelPtr>;
}  // namespace view_model
}  // namespace urdf_builder
}  // namespace gui
