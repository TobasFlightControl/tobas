// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QSettings>
#include <eigen3/Eigen/Core>

#include <tobas_qt_tools/widgets/table_widget.hpp>

#include "./base.hpp"

namespace tobas::gui::sa
{
class ParamGetterWidget_DoubleTable : public ParamGetterWidget<Eigen::MatrixXd>
{
  Q_OBJECT

  using self = ParamGetterWidget_DoubleTable;
  using super = ParamGetterWidget<Eigen::MatrixXd>;

Q_SIGNALS:
  void dataChanged();

public:
  explicit ParamGetterWidget_DoubleTable(
    const QString& _param_name,
    const QString& _title,
    const QStringList& _labels,
    const QString& _description_text = "");

  Eigen::MatrixXd getValue() const override;
  bool setValue(const Eigen::MatrixXd& _src) override;

  qt::TableWidget* table();

  void setDecimals(const QVector<int>& _decimals);
  void setMinimum(const QVector<double>& _minimum);
  void setMaximum(const QVector<double>& _maximum);
  void setDefault(const QVector<double>& _default);

  int count() const;

private Q_SLOTS:
  void addRow();
  void deleteRow();
  void loadCsv();
  void onCellValueChanged();

private:
  const QString last_opened_dir_key_;
  const QString title_;
  const QStringList labels_;
  const int num_entry_;

  QVector<double> minimum_;
  QVector<double> maximum_;
  QVector<double> default_;
  QVector<int> decimals_;

  qt::TableWidget* table_;

  QSettings settings_store_;

  QString getCsvPath();
  bool isValidData(const Eigen::MatrixXd& _src);
};
}  // namespace tobas::gui::sa
