#include "../../include/urdf_builder/view_model/collision_view_model.hpp"
#include "../../include/urdf_builder/utils/time.hpp"

namespace urdf_builder
{
namespace view_model
{
CollisionViewModel::CollisionViewModel(const urdf::CollisionSharedPtr& model)
  : BaseViewModel<urdf::Collision, CollisionViewModel>(model),
    geometry_vm_(std::make_shared<GeometryViewModel>(model_->geometry))
{
  if (model_->name.empty())
    model_->name = "collision_" + std::to_string(utils::timeNowMilliseconds());
}

const urdf::Pose& CollisionViewModel::origin() const
{
  return model_->origin;
}

void CollisionViewModel::origin(const urdf::Pose& origin)
{
  model_->origin = origin;
}

QString CollisionViewModel::name() const
{
  return QString::fromStdString(model_->name);
}

void CollisionViewModel::name(const QString& name)
{
  model_->name = name.toStdString();
}

const GeometryViewModelPtr& CollisionViewModel::geometry() const
{
  return geometry_vm_;
}

void CollisionViewModel::sync()
{
  geometry_vm_->sync();
  model_->geometry = geometry_vm_->model();
}
}  // namespace view_model
}  // namespace urdf_builder
