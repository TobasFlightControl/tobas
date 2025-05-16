#include "tobas_urdf_builder_plugin/view_model/visual_view_model.hpp"

#include "tobas_urdf_builder_plugin/utils/time.hpp"

namespace gui
{
namespace urdf_builder
{
namespace view_model
{
VisualViewModel::VisualViewModel(const urdf::VisualSharedPtr& model)
  : BaseViewModel<urdf::Visual, VisualViewModel>(model)
  , geometry_vm_(std::make_shared<GeometryViewModel>(model_->geometry))
  , material_vm_(std::make_shared<MaterialViewModel>(model_->material))
{
  if (model_->name.empty()) {
    model_->name = "visual_" + std::to_string(utils::timeNowMilliseconds());
  }
}

void VisualViewModel::sync()
{
  geometry_vm_->sync();
  model_->geometry = geometry_vm_->model();
  model_->material = material_vm_->model();
}

QString VisualViewModel::name() const
{
  return QString::fromStdString(model_->name);
}

void VisualViewModel::name(const QString& name)
{
  model_->name = name.toStdString();
}

const urdf::Pose& VisualViewModel::origin() const
{
  return model_->origin;
}

void VisualViewModel::origin(const urdf::Pose& origin)
{
  model_->origin = origin;
}

const GeometryViewModelPtr& VisualViewModel::geometry()
{
  return geometry_vm_;
}

const MaterialViewModelPtr& VisualViewModel::material()
{
  return material_vm_;
}
}  // namespace view_model
}  // namespace urdf_builder
}  // namespace gui
