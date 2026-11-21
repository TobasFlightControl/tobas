// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QFileDialog>
#include <QWidget>
#include <eigen3/Eigen/Core>

#include <tobas_property_client/property_client.hpp>
#include <tobas_qt_tools/widgets/table_widget.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
class DoubleTableWidget : public QWidget
{
  Q_OBJECT

  using self = DoubleTableWidget;

  static constexpr int kButtonWidth = 96;
  static constexpr double kDefaultValue = 0.0;
  static constexpr int kDefaultDecimals = 2;

Q_SIGNALS:
  void dataChanged();

public:
  explicit DoubleTableWidget(
    rclcpp::Node::SharedPtr node,
    const QString& param_name,
    const QString& title,
    const QStringList& labels,
    const QString& description_text = "");

  Eigen::MatrixXd getData() const;
  bool setData(const Eigen::MatrixXd& src);

  qt::TableWidget* table();

  void setDecimals(const QVector<int>& decimals);
  void setMinimum(const QVector<double>& minimum);
  void setMaximum(const QVector<double>& maximum);
  void setDefault(const QVector<double>& _default);

  int count() const;

private Q_SLOTS:
  void addRow();
  void deleteRow();
  void loadCsv();
  void onCellValueChanged();

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string last_opend_dir_key_;
  const QString title_;
  const QStringList labels_;
  const int num_entry_;

  QVector<double> minimum_;
  QVector<double> maximum_;
  QVector<double> default_;
  QVector<int> decimals_;

  qt::TableWidget* table_;

  ptree::PropertyClient property_client_;

  QString getCsvPath();
  bool isValidData(const Eigen::MatrixXd& src);
  void saveLastOpenedDir(const std::string& dir);
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
