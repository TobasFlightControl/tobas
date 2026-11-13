// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/param_getters/vector3d.hpp"

#include <tobas_string_tools/core.hpp>

namespace tobas::gui::sa
{
ParamGetterWidget_Vector3d::ParamGetterWidget_Vector3d(const QString& _param_name, const QString& _description_text)
  : super(_param_name, _description_text)
{
  vector3d_ = new qt::Vector3dEditHorizontal();
  rows_->addWidget(vector3d_);
  connect(vector3d_, &qt::Vector3dEditHorizontal::valueChanged, this, &self::onValueChanged);
}

Eigen::Vector3d ParamGetterWidget_Vector3d::getValue() const
{
  return vector3d_->vector();
}

bool ParamGetterWidget_Vector3d::setValue(const Eigen::Vector3d& _src)
{
  vector3d_->setVector(_src);
  return true;
}

void ParamGetterWidget_Vector3d::setDecimals(int _decimals)
{
  vector3d_->setDecimals(_decimals);
}

void ParamGetterWidget_Vector3d::setMinimum(double _minimum)
{
  vector3d_->setMinimum(_minimum);
}

void ParamGetterWidget_Vector3d::setMaximum(double _maximum)
{
  vector3d_->setMaximum(_maximum);
}

void ParamGetterWidget_Vector3d::setSingleStep(double _single_step)
{
  vector3d_->setSingleStep(_single_step);
}

void ParamGetterWidget_Vector3d::setSuffix(const QString& _suffix)
{
  vector3d_->setSuffix(QString::fromStdString(str::convertToSuperscript(_suffix.toStdString())));
}

void ParamGetterWidget_Vector3d::onValueChanged(const Eigen::Vector3d& _value)
{
  Q_EMIT valueChanged(_value);
}
}  // namespace tobas::gui::sa
