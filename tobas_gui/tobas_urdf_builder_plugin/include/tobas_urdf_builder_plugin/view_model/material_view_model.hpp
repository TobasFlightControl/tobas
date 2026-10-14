// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QColor>
#include <QString>

#include "../utils/constants.hpp"
#include "./base_view_model.hpp"

namespace tobas
{
namespace gui
{
namespace ub
{
namespace view_model
{
class MaterialViewModel : public BaseViewModel<::urdf::Material, MaterialViewModel>
{
public:
  explicit MaterialViewModel(const ::urdf::MaterialSharedPtr& _model);

  void sync() override;

  QString name() const;
  void name(const QString& _name);

  const ::urdf::Color& color() const;
  void color(double _r, double _g, double _b, double _a = kDefaultRobotAlpha);
  void color(const QColor& _color);

  QString textureFileName() const;
  void textureFileName(const QString& _filename);
};

using MaterialViewModelPtr = std::shared_ptr<MaterialViewModel>;
}  // namespace view_model
}  // namespace ub
}  // namespace gui
}  // namespace tobas
