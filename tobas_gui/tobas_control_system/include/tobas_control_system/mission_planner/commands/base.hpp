// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>
#include <QMap>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

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

  static constexpr int kLablePSize = 12;
  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

Q_SIGNALS:
  void updated();
  void deleteButtonClicked();

public:
  explicit BaseCommandWidget();

  virtual const char* name() const = 0;

protected:
  void addField(field::BaseFieldWidget* widget, bool overridable = false);

  bool isChecked(field::BaseFieldWidget* widget) const;

  /* チェックされていたら設定された値，されていなければゼロを返す． */
  template <typename T>
  T getValue(field::FieldWidget<T>* widget) const;

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

template <typename T>
T BaseCommandWidget::getValue(field::FieldWidget<T>* widget) const
{
  return isChecked(widget) ? widget->getValue() : static_cast<T>(0);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
