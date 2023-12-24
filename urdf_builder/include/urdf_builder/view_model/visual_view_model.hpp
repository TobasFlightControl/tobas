#pragma once

#include <memory>

#include "./base_view_model.hpp"
#include "../utils/time.hpp"

namespace urdf_builder
{
namespace view_model
{
class VisualViewModel : public BaseViewModel<urdf::Visual, VisualViewModel>
{
public:
  explicit VisualViewModel(const urdf::VisualSharedPtr& model)
    : BaseViewModel<urdf::Visual, VisualViewModel>(model),
      geometry_vm_(std::make_shared<GeometryViewModel>(model_->geometry)),
      material_vm_(std::make_shared<MaterialViewModel>(model_->material))
  {
    if (model_->name.empty())
      model_->name = "visual_" + std::to_string(utils::timeNowMilliseconds());
  }

  QString name() const
  {
    return QString::fromStdString(model_->name);
  }

  void name(const QString& name)
  {
    model_->name = name.toStdString();
  }

  const urdf::Pose& origin() const
  {
    return model_->origin;
  }

  void origin(const urdf::Pose& origin)
  {
    model_->origin = origin;
  }

  const MaterialViewModelPtr& material() const
  {
    return material_vm_;
  }

  const GeometryViewModelPtr& geometry() const
  {
    return geometry_vm_;
  }

  void sync() override
  {
    geometry_vm_->sync();
    model_->geometry = geometry_vm_->model();
    model_->material = material_vm_->model();
  }

private:
  GeometryViewModelPtr geometry_vm_;
  MaterialViewModelPtr material_vm_;
};

using VisualViewModelPtr = std::shared_ptr<VisualViewModel>;
using V_VisualViewModelPtr = std::vector<VisualViewModelPtr>;
}  // namespace view_model
}  // namespace urdf_builder
