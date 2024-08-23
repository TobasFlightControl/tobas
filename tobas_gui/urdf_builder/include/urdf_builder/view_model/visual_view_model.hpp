#pragma once

#include "./base_view_model.hpp"
#include "./geometry_view_model.hpp"
#include "./material_view_model.hpp"

namespace urdf_builder
{
namespace view_model
{
class VisualViewModel : public BaseViewModel<urdf::Visual, VisualViewModel>
{
public:
  explicit VisualViewModel(const urdf::VisualSharedPtr& model);

  void sync() override;

  QString name() const;
  void name(const QString& name);

  const urdf::Pose& origin() const;
  void origin(const urdf::Pose& origin);

  const GeometryViewModelPtr& geometry();
  const MaterialViewModelPtr& material();

private:
  GeometryViewModelPtr geometry_vm_;
  MaterialViewModelPtr material_vm_;
};

using VisualViewModelPtr = std::shared_ptr<VisualViewModel>;
using V_VisualViewModelPtr = std::vector<VisualViewModelPtr>;
}  // namespace view_model
}  // namespace urdf_builder
