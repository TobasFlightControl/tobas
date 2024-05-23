#pragma once

#include <QtGui/QtGui>

#include "./base_view_model.hpp"
#include "../utils/time.hpp"
#include "../utils/constants.hpp"

namespace urdf_builder
{
namespace view_model
{
class MaterialViewModel : public BaseViewModel<urdf::Material, MaterialViewModel>
{
  static constexpr float kDefaultColorR = 1.;
  static constexpr float kDefaultColorG = 1.;
  static constexpr float kDefaultColorB = 1.;

public:
  explicit MaterialViewModel(const urdf::MaterialSharedPtr& model);

  QString name() const;
  void name(const QString& name);

  const urdf::Color& color() const;
  void color(double r, double g, double b, double a = kDefaultRobotAlpha);
  void color(const QColor& _color);

  QString textureFileName() const;
  void textureFileName(const QString& filename);
};

using MaterialViewModelPtr = std::shared_ptr<MaterialViewModel>;
}  // namespace view_model
}  // namespace urdf_builder
