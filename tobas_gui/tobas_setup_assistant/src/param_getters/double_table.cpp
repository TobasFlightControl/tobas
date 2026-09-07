// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/param_getters/double_table.hpp"

#include <QDir>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_std_tools/check.hpp>

#include "tobas_setup_assistant/rapidcsv.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
ParamGetterWidget_DoubleTable::ParamGetterWidget_DoubleTable(
  const QString& param_name,
  const QString& title,
  const QStringList& labels,
  const QString& description_text)
  : super(param_name, description_text)
  , last_opened_dir_key_("double_table/last_opened_dir/" + QString(param_name).replace(' ', '_'))
  , title_(title)
  , labels_(labels)
  , num_entry_(labels.size())
{
  TOBAS_CHECK(num_entry_ > 0);

  minimum_.fill(std::numeric_limits<double>::lowest(), num_entry_);
  maximum_.fill(std::numeric_limits<double>::max(), num_entry_);
  default_.fill(0.0, num_entry_);
  decimals_.fill(2, num_entry_);

  const auto add_row_btn = new QPushButton("Add Row");
  const auto delete_row_btn = new QPushButton("Delete Row");
  const auto clear_btn = new QPushButton("Clear");
  const auto load_csv_btn = new QPushButton("Load CSV");

  constexpr int kButtonWidth = 90;
  constexpr int kButtonHeight = 36;
  add_row_btn->setFixedSize(kButtonWidth, kButtonHeight);
  delete_row_btn->setFixedSize(kButtonWidth, kButtonHeight);
  clear_btn->setFixedSize(kButtonWidth, kButtonHeight);
  load_csv_btn->setFixedSize(kButtonWidth, kButtonHeight);

  table_ = new qt::TableWidget(0, num_entry_);
  table_->verticalHeader()->setVisible(true);  // Show row numbers.
  table_->setHorizontalHeaderLabels(labels);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(add_row_btn);
  cols->addWidget(delete_row_btn);
  cols->addWidget(clear_btn);
  cols->addWidget(load_csv_btn);
  cols->addStretch();  // Left-align the button.

  rows_->addLayout(cols);
  rows_->addWidget(table_);

  // Connection
  connect(add_row_btn, &QPushButton::clicked, this, &self::addRow);
  connect(delete_row_btn, &QPushButton::clicked, this, &self::deleteRow);
  connect(clear_btn, &QPushButton::clicked, table_, &qt::TableWidget::removeAll);
  connect(load_csv_btn, &QPushButton::clicked, this, &self::loadCsv);
}

Eigen::MatrixXd ParamGetterWidget_DoubleTable::getValue() const
{
  const auto rows = count();

  Eigen::MatrixXd res(rows, num_entry_);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < num_entry_; ++col) {
      const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(table_->cellWidget(row, col));
      res(row, col) = cell->value();
    }
  }

  return res;
}

bool ParamGetterWidget_DoubleTable::setValue(const Eigen::MatrixXd& src)
{
  if (!isValidData(src)) {
    return false;
  }

  table_->removeAll();

  for (int row = 0; row < src.rows(); ++row) {
    addRow();
    for (int col = 0; col < src.cols(); ++col) {
      const auto cell = qt::qPointerCast<qt::DoubleSpinBox>(table_->cellWidget(row, col));
      cell->setValue(src(row, col));
    }
  }

  return true;
}

qt::TableWidget* ParamGetterWidget_DoubleTable::table()
{
  return table_;
}

void ParamGetterWidget_DoubleTable::setDecimals(const QVector<int>& decimals)
{
  TOBAS_CHECK(decimals.size() == num_entry_);
  decimals_ = decimals;
}

void ParamGetterWidget_DoubleTable::setMinimum(const QVector<double>& minimum)
{
  TOBAS_CHECK(minimum.size() == num_entry_);
  minimum_ = minimum;
}

void ParamGetterWidget_DoubleTable::setMaximum(const QVector<double>& maximum)
{
  TOBAS_CHECK(maximum.size() == num_entry_);
  maximum_ = maximum;
}

void ParamGetterWidget_DoubleTable::setDefault(const QVector<double>& _default)
{
  TOBAS_CHECK(default_.size() == num_entry_);
  default_ = _default;
}

int ParamGetterWidget_DoubleTable::count() const
{
  return table_->rowCount();
}

void ParamGetterWidget_DoubleTable::addRow()
{
  const auto rows = count();
  table_->insertRow(rows);

  for (int col = 0; col < num_entry_; ++col) {
    const auto cell = new qt::DoubleSpinBox();
    cell->setButtonSymbols(QAbstractSpinBox::NoButtons);  // Remove spin buttons.
    cell->setMinimum(minimum_[col]);
    cell->setMaximum(maximum_[col]);
    cell->setValue(default_[col]);
    cell->setDecimals(decimals_[col]);
    connect(cell, qOverload<double>(&qt::DoubleSpinBox::valueChanged), this, &self::onCellValueChanged);
    table_->setCellWidget(rows, col, cell);
  }
}

void ParamGetterWidget_DoubleTable::deleteRow()
{
  const auto row = table_->currentRow();
  if (row < 0) {
    return;
  }
  table_->removeRow(row);
}

void ParamGetterWidget_DoubleTable::loadCsv()
{
  // Get CSV file path.
  const auto file_path = getCsvPath();
  if (file_path.isEmpty()) {
    return;
  }

  // Load CSV.
  const auto doc = csv::load(file_path.toStdString());

  // Read data.
  std::vector<std::vector<double>> columns(num_entry_);
  for (int i = 0; i < num_entry_; ++i) {
    const auto& label = labels_.at(i);
    if (!csv::getColumn(doc, label.toStdString(), columns[i])) {
      qt::qErrorBox(this, "Failed to get column: " + label);
      return;
    }
  }

  // Fill data.
  const auto num_data = columns.front().size();
  Eigen::MatrixXd data_array(num_data, num_entry_);
  for (int col = 0; col < num_entry_; ++col) {
    if (columns.at(col).size() != num_data) {
      qt::qErrorBox(this, "Data size mismatch.");
      return;
    }
    for (size_t row = 0; row < num_data; ++row) {
      data_array(row, col) = columns.at(col).at(row);
    }
  }

  // Set data.
  if (!setValue(data_array)) {
    return;
  }

  qt::qInfoBox(this, "Data is loaded successfully.");
}

void ParamGetterWidget_DoubleTable::onCellValueChanged()
{
  Q_EMIT dataChanged();
}

QString ParamGetterWidget_DoubleTable::getCsvPath()
{
  const auto last_opened_dir = settings_store_.value(last_opened_dir_key_, QDir::homePath()).toString();

  const auto file_path = QFileDialog::getOpenFileName(
    this, title_, last_opened_dir, "CSV File (*.csv)", nullptr, QFileDialog::DontUseNativeDialog);

  // Save the last opened path.
  if (!file_path.isEmpty()) {
    const auto par_dir = QFileInfo(file_path).absolutePath();
    settings_store_.setValue(last_opened_dir_key_, par_dir);
  }

  return file_path;
}

bool ParamGetterWidget_DoubleTable::isValidData(const Eigen::MatrixXd& src)
{
  if (src.cols() != num_entry_) {
    qt::qErrorBox(this, "Column size mismatch.");
    return false;
  }

  for (int col = 0; col < num_entry_; ++col) {
    const auto column = src.col(col).array().eval();
    if ((column < minimum_.at(col)).any() || (maximum_.at(col) < column).any()) {
      qt::qErrorBox(this, "Some values of field '" + labels_.at(col) + "' are out of limits.");
      return false;
    }
  }

  return true;
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
