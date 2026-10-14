// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QButtonGroup>
#include <QRadioButton>

#include <tobas_qt_tools/layouts/form_layout.hpp>

#include "./base.hpp"

namespace tobas::gui::sim
{
class WorldWidget : public QWidget
{
  Q_OBJECT

  using self = WorldWidget;

public:
  explicit WorldWidget();

  std::filesystem::path worldPath() const;

private:
  QVector<QRadioButton*> buttons_;
  QVector<BaseWorldWidget*> widgets_;

  void addRow(qt::FormLayout* _form, QButtonGroup* _btn_group, BaseWorldWidget* _widget, const QString& _label);

  void updateEnabled();

  int rowCount() const;
  int findCurrentRow() const;

private Q_SLOTS:
  void onButtonGroupIdClicked();
};
}  // namespace tobas::gui::sim
