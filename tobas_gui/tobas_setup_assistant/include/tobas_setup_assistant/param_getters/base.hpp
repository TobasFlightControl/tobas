// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QFormLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
/* Base class for widgets that retrieve user parameters. */
template <typename T>
class ParamGetterWidget : public QWidget
{
public:
  using ValueType = T;

  explicit ParamGetterWidget(const QString& param_name, const QString& description_text);

  virtual T getValue() const = 0;
  virtual bool setValue(const T& src) = 0;

  std::string name() const;

protected:
  QVBoxLayout* rows_;
  QLabel* label_;
};

template <typename T>
ParamGetterWidget<T>::ParamGetterWidget(const QString& param_name, const QString& description_text)
{
  rows_ = new QVBoxLayout();
  rows_->setContentsMargins(0, 0, 0, 0);
  rows_->setSpacing(4);

  label_ = new QLabel(param_name);
  label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  label_->setToolTip(description_text);
  setToolTip(description_text);

  const auto field = new QWidget();
  field->setLayout(rows_);
  const auto form = new QFormLayout(this);
  form->setContentsMargins(0, 0, 0, 0);
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  form->addRow(label_, field);
}

template <typename T>
std::string ParamGetterWidget<T>::name() const
{
  return label_->text().toStdString();
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
