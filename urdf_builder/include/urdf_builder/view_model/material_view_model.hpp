#pragma once

#include <QColor>

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
  explicit MaterialViewModel(const urdf::MaterialSharedPtr& model)
    : BaseViewModel<urdf::Material, MaterialViewModel>(model)
  {
    if (model_->name.empty())
    {
      // Set default name
      model_->name = "material_" + std::to_string(utils::timeNowMilliseconds());

      // Set default color
      model_->color.r = kDefaultColorR;
      model_->color.g = kDefaultColorG;
      model_->color.b = kDefaultColorB;
      model_->color.a = kDefaultRobotAlpha;
    }
  }

  QString name() const
  {
    return QString::fromStdString(model_->name);
  }

  void name(const QString& name)
  {
    model_->name = name.toStdString();
  }

  const urdf::Color& color() const
  {
    return model_->color;
  }

  void color(double r, double g, double b, double a = kDefaultRobotAlpha)
  {
    model_->color.r = static_cast<float>(r);
    model_->color.g = static_cast<float>(g);
    model_->color.b = static_cast<float>(b);
    model_->color.a = static_cast<float>(a);
  }

  void color(const QColor& _color)
  {
    color(_color.redF(), _color.greenF(), _color.blueF(), _color.alphaF());
  }

  QString textureFileName() const
  {
    return QString::fromStdString(model_->texture_filename);
  }

  void textureFileName(const QString& filename)
  {
    model_->texture_filename = filename.toStdString();
  }
};

using MaterialViewModelPtr = std::shared_ptr<MaterialViewModel>;
}  // namespace view_model
}  // namespace urdf_builder
