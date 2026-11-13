// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <map>

#include <QDialog>
#include <QDoubleSpinBox>

namespace tobas::gui::ub::ui
{
class DoubleMapInputDialog : public QDialog
{
  Q_OBJECT

  using self = DoubleMapInputDialog;
  using super = QDialog;

public:
  explicit DoubleMapInputDialog(QWidget* _parent, const QString& _title, const QStringList& _field_names);

  const double& getValue(const QString& _field) const;

private Q_SLOTS:
  void onSpinBoxValueChanged(double _value);

private:
  std::map<QString, double> field2value_;
  std::map<QDoubleSpinBox*, QString> spinbox2field_;
};
}  // namespace tobas::gui::ub::ui
