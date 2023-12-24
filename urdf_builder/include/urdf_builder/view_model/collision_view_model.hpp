#pragma once

#include <QString>

#include "./base_view_model.hpp"
#include "./geometry_view_model.hpp"
#include "../utils/time.hpp"

namespace urdf_builder
{
namespace view_model
{
class CollisionViewModel : public BaseViewModel<urdf::Collision, CollisionViewModel>
{
public:
  explicit CollisionViewModel(const urdf::CollisionSharedPtr& model)
    : BaseViewModel<urdf::Collision, CollisionViewModel>(model),
      geometry_vm_(std::make_shared<GeometryViewModel>(model_->geometry))
  {
    if (model_->name.empty())
      model_->name = "collision_" + std::to_string(utils::timeNowMilliseconds());
  }

  const urdf::Pose& origin() const
  {
    return model_->origin;
  }

  void origin(const urdf::Pose& origin)
  {
    model_->origin = origin;
  }

  QString name() const
  {
    return QString::fromStdString(model_->name);
  }

  void name(const QString& name)
  {
    model_->name = name.toStdString();
  }

  const GeometryViewModelPtr& geometry() const
  {
    return geometry_vm_;
  }

  void sync() override
  {
    geometry_vm_->sync();
    model_->geometry = geometry_vm_->model();
  }

private:
  GeometryViewModelPtr geometry_vm_;
};

using CollisionViewModelPtr = std::shared_ptr<CollisionViewModel>;
using V_CollisionViewModelPtr = std::vector<CollisionViewModelPtr>;
}  // namespace view_model
}  // namespace urdf_builder
