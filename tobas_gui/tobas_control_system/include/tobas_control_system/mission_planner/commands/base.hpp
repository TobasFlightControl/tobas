// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>
#include <QMap>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include <tobas_mission_items/mission_items.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/widgets/check_box.hpp>

#include "../fields/base.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
class BaseCommandWidget : public QWidget
{
  Q_OBJECT

  using self = BaseCommandWidget;
  using super = QWidget;

Q_SIGNALS:
  void updated();
  void deleteButtonClicked();

public:
  explicit BaseCommandWidget();

  virtual const char* name() const = 0;

protected:
  void addField(field::BaseFieldWidget* widget, bool overridable = false);

  bool isChecked(field::BaseFieldWidget* widget) const;
  void setChecked(field::BaseFieldWidget* widget, bool checked);

  /* Return the configured value if checked; otherwise return NaN. */
  double getValueOrDefault(field::FieldWidget<double>* widget) const;
  /* Set a valid value as-is; use the default value if it is NaN. */
  void setValueOrDefault(field::FieldWidget<double>* widget, double value);

private:
  QLabel* label_;
  QPushButton* delete_button_;
  qt::FormLayout* form_;
  QMap<field::BaseFieldWidget*, qt::CheckBox*> checkboxes_;

  int row_ = 0;

private Q_SLOTS:
  void initialize();
  void onFieldUpdated();
  void onDeleteButtonClicked();
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
