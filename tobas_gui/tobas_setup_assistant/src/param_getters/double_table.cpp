#include "tobas_setup_assistant/param_getters/double_table.hpp"

#include <filesystem>

#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_string_tools/core.hpp>

#include "tobas_setup_assistant/rapidcsv.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace sa
{
ParamGetterWidget_DoubleTable::ParamGetterWidget_DoubleTable(
  rclcpp::Node::SharedPtr node,
  const QString& param_name,
  const QString& title,
  const QStringList& labels,
  const QString& description_text)
  : super(param_name, description_text)
  , node_(node)
  , last_opend_dir_key_("last_opened_dir/" + str::replace(param_name.toStdString(), " ", "_"))
  , title_(title)
  , labels_(labels)
  , num_entry_(labels.size())
  , property_client_(node, "tobas_setup_assistant/double_table")
{
  TOBAS_CHECK(num_entry_ > 0);

  minimum_.fill(std::numeric_limits<double>::lowest(), num_entry_);
  maximum_.fill(std::numeric_limits<double>::max(), num_entry_);
  default_.fill(kDefaultValue, num_entry_);
  decimals_.fill(kDefaultDecimals, num_entry_);

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
    cell->setButtonSymbols(QAbstractSpinBox::NoButtons);  // 増減ボタンを削除
    cell->setMinimum(minimum_[col]);
    cell->setMaximum(maximum_[col]);
    cell->setValue(default_[col]);
    cell->setDecimals(decimals_[col]);
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

void ParamGetterWidget_DoubleTable::loadCsv()
{
  // Get CSV file path
  const auto file_path = getCsvPath();
  if (file_path.isEmpty()) {
    return;
  }

  // Load CSV
  const auto doc = csv::load(file_path.toStdString());

  // Read data
  std::vector<std::vector<double>> columns(num_entry_);
  for (int i = 0; i < num_entry_; ++i) {
    const auto& label = labels_.at(i);
    if (!csv::getColumn(doc, label.toStdString(), columns[i])) {
      qt::qErrorBox(this, "Failed to get column: " + label);
      return;
    }
  }

  // Fill data
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

QString ParamGetterWidget_DoubleTable::getCsvPath()
{
  std::string last_opened_dir;
  if (property_client_.get(last_opend_dir_key_, last_opened_dir) < 0) {
    RCLCPP_WARN_STREAM(node_->get_logger(), property_client_.errorMessage());
    last_opened_dir = ros2::getHomeDir();
  }

  const auto file_path = QFileDialog::getOpenFileName(
    this, title_, QString::fromStdString(last_opened_dir), "CSV File (*.csv)", nullptr, QFileDialog::DontUseNativeDialog);

  // 最後に開かれたパスを保存
  if (!file_path.isEmpty()) {
    const auto par_dir = fs::path(file_path.toStdString()).parent_path();
    saveLastOpenedDir(par_dir);
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
      qt::qErrorBox(this, "Some values of field \"" + labels_.at(col) + "\" are out of limits.");
      return false;
    }
  }

  return true;
}

void ParamGetterWidget_DoubleTable::saveLastOpenedDir(const std::string& dir)
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
