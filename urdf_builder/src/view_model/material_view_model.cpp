#include "../../include/urdf_builder/view_model/material_view_model.hpp"

namespace urdf_builder
{
namespace view_model
{
MaterialViewModel::MaterialViewModel(const urdf::MaterialSharedPtr& model)
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

QString MaterialViewModel::name() const
{
  return QString::fromStdString(model_->name);
}

void MaterialViewModel::name(const QString& name)
{
  model_->name = name.toStdString();
}

const urdf::Color& MaterialViewModel::color() const
{
  return model_->color;
}

void MaterialViewModel::color(double r, double g, double b, double a)
{
  model_->color.r = static_cast<float>(r);
  model_->color.g = static_cast<float>(g);
  model_->color.b = static_cast<float>(b);
  model_->color.a = static_cast<float>(a);
}

void MaterialViewModel::color(const QColor& _color)
{
  color(_color.redF(), _color.greenF(), _color.blueF(), _color.alphaF());
}

QString MaterialViewModel::textureFileName() const
{
  return QString::fromStdString(model_->texture_filename);
}

void MaterialViewModel::textureFileName(const QString& filename)
{
  model_->texture_filename = filename.toStdString();
}
}  // namespace view_model
}  // namespace urdf_builder
