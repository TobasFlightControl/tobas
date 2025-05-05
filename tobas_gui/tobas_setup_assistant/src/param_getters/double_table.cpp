#include "tobas_setup_assistant/param_getters/double_table.hpp"

#include <rapidcsv.h>
#include <rcutils/env.h>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>
#include <filesystem>

#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_string_tools/core.hpp>

using namespace std;
using namespace Eigen;

namespace gui
{
namespace sa
{
ParamGetterWidget_DoubleTable::ParamGetterWidget_DoubleTable(
  rclcpp::Node::SharedPtr node,
  const QString& param_name,
  const QStringList& labels,
  const QString& description_text)
  : super(param_name, description_text)
  , node_(node)
  , last_opend_dir_key_("last_opened_dir/double_table/" + str::replace(param_name.toStdString(), " ", "_"))
  , labels_(labels)
  , num_entry_(labels.size())
  , property_client_(node, tobas::kPropertyServerName, kPackageName)
{
  TOBAS_CHECK(num_entry_ > 0);

  minimum_.fill(numeric_limits<double>::lowest(), num_entry_);
  maximum_.fill(numeric_limits<double>::max(), num_entry_);
  default_.fill(kDefaultValue, num_entry_);
  decimals_.fill(kDefaultDecimals, num_entry_);
  suffix_.fill("", num_entry_);

  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  const auto add_row_btn = new QPushButton("Add Row");
  add_row_btn->setFixedSize(kButtonWidth, kButtonHeight);
  cols->addWidget(add_row_btn);

  const auto delete_row_btn = new QPushButton("Delete Row");
  delete_row_btn->setFixedSize(kButtonWidth, kButtonHeight);
  cols->addWidget(delete_row_btn);

  const auto clear_btn = new QPushButton("Clear");
  clear_btn->setFixedSize(kButtonWidth, kButtonHeight);
  cols->addWidget(clear_btn);

  const auto load_csv_btn = new QPushButton("Load CSV");
  load_csv_btn->setFixedSize(kButtonWidth, kButtonHeight);
  cols->addWidget(load_csv_btn);

  cols->addStretch();  // ボタンを左詰めにする

  table_ = new qt::TableWidget(0, num_entry_);
  table_->verticalHeader()->setVisible(true);  // 行番号を表示
  table_->setHorizontalHeaderLabels(labels);
  rows_->addWidget(table_);

  // Connection
  connect(add_row_btn, &QPushButton::clicked, this, &self::addRow);
  connect(delete_row_btn, &QPushButton::clicked, this, &self::deleteRow);
  connect(clear_btn, &QPushButton::clicked, table_, &qt::TableWidget::removeAll);
  connect(load_csv_btn, &QPushButton::clicked, this, &self::loadCSV);
}

MatrixXd ParamGetterWidget_DoubleTable::getValue() const
{
  const auto rows = count();

  MatrixXd res(rows, num_entry_);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < num_entry_; ++col) {
      const auto cell = qt::qConstPointerCast<qt::DoubleSpinBox>(table_->cellWidget(row, col));
      res(row, col) = cell->value();
    }
  }

  return res;
}

bool ParamGetterWidget_DoubleTable::setValue(const MatrixXd& src)
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

void ParamGetterWidget_DoubleTable::setSuffix(const QVector<QString>& suffix)
{
  TOBAS_CHECK(suffix.size() == num_entry_);
  suffix_ = suffix;
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
    cell->setMinimum(minimum_[col]);
    cell->setMaximum(maximum_[col]);
    cell->setValue(default_[col]);
    cell->setDecimals(decimals_[col]);
    cell->setSuffix(suffix_[col]);
    connect(cell, QOverload<double>::of(&qt::DoubleSpinBox::valueChanged), this, &self::onCellValueChanged);
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

void ParamGetterWidget_DoubleTable::loadCSV()
{
  // Get CSV file path
  const auto file_path = getCSVFilePath();
  if (file_path.isEmpty())  // Cancelの場合
  {
    return;
  }

  // Load CSV
  rapidcsv::Document doc(file_path.toStdString());

  // Read data
  vector<vector<double>> columns(num_entry_);
  for (int i = 0; i < num_entry_; ++i) {
    const auto& label = labels_.at(i);
    try {
      columns[i] = doc.GetColumn<double>(label.toStdString());
    }
    catch (...) {
      qt::qErrorBox(this, "Field \"" + label + "\" does not exist.");
      return;
    }
  }

  // Fill data
  const size_t num_data = columns.front().size();
  MatrixXd data_array(num_data, num_entry_);
  for (int col = 0; col < num_entry_; ++col) {
    if (columns.at(col).size() != num_data) {
      qt::qErrorBox(this, "Data size mismatch.");
      return;
    }
    for (size_t row = 0; row < num_data; ++row) {
      data_array(row, col) = columns.at(col).at(row);
    }
  }

  // Set data
  if (!setValue(data_array)) {
    return;
  }

  qt::qInfoBox(this, "Data is loaded successfully.");
}

void ParamGetterWidget_DoubleTable::onCellValueChanged()
{
  Q_EMIT dataChanged();
}

QString ParamGetterWidget_DoubleTable::getCSVFilePath()
{
  string last_opened_dir;
  if (property_client_.get(last_opend_dir_key_, last_opened_dir) < 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    last_opened_dir = rcutils_get_home_dir();
  }

  const auto options = QFileDialog::DontUseNativeDialog;
  const auto file_path = QFileDialog::getOpenFileName(
    this, kTitle, QString::fromStdString(last_opened_dir), "CSV File (*.csv)", nullptr, options);

  // 最後に開かれたパスを保存
  if (!file_path.isEmpty()) {
    const auto par_dir = filesystem::path(file_path.toStdString()).parent_path();
    saveLastOpenedDir(par_dir);
  }

  return file_path;
}

bool ParamGetterWidget_DoubleTable::isValidData(const MatrixXd& src)
{
  if (src.cols() != num_entry_) {
    qt::qErrorBox(this, "Column size mismatch.");
    return false;
  }

  for (int col = 0; col < num_entry_; ++col) {
    const ArrayXd column = src.col(col).array();
    if ((column < minimum_.at(col)).any() || (maximum_.at(col) < column).any()) {
      qt::qErrorBox(this, "Some values of field \"" + labels_.at(col) + "\" are out of limits.");
      return false;
    }
  }

  return true;
}

void ParamGetterWidget_DoubleTable::saveLastOpenedDir(const string& dir)
{
  if (property_client_.set(last_opend_dir_key_, dir) < 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    return;
  }
  if (property_client_.save() < 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    return;
  }
}
}  // namespace sa
}  // namespace gui
