// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/view_model/material_view_model.hpp"

#include "tobas_urdf_builder_plugin/utils/time.hpp"

namespace tobas::gui::ub::view_model
{
MaterialViewModel::MaterialViewModel(const ::urdf::MaterialSharedPtr& _model)
  : BaseViewModel<::urdf::Material, MaterialViewModel>(_model)
{
  if (model_->name.empty()) {
    // Set default name.
    model_->name = "material_" + std::to_string(utils::timeNowMilliseconds());

    // Set default color.
    model_->color.r = 1.0f;
    model_->color.g = 1.0f;
    model_->color.b = 1.0f;
    model_->color.a = kDefaultRobotAlpha;
  }
}

void MaterialViewModel::sync()
{
}

QString MaterialViewModel::name() const
{
  return QString::fromStdString(model_->name);
}

void MaterialViewModel::name(const QString& _name)
{
  model_->name = _name.toStdString();
}

const ::urdf::Color& MaterialViewModel::color() const
{
  return model_->color;
}

void MaterialViewModel::color(double _r, double _g, double _b, double _a)
{
  model_->color.r = static_cast<float>(_r);
  model_->color.g = static_cast<float>(_g);
  model_->color.b = static_cast<float>(_b);
  model_->color.a = static_cast<float>(_a);
}

void MaterialViewModel::color(const QColor& _color)
{
  color(_color.redF(), _color.greenF(), _color.blueF(), _color.alphaF());
}

QString MaterialViewModel::textureFileName() const
{
  return QString::fromStdString(model_->texture_filename);
}

void MaterialViewModel::textureFileName(const QString& _filename)
{
  model_->texture_filename = _filename.toStdString();
}
}  // namespace tobas::gui::ub::view_model
