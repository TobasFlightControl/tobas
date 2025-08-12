#include "tobas_bootmedia_config/wifi_client.hpp"

#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_bootmedia_config/constants.hpp"

namespace gui
{
namespace bm
{
WifiClientWidget::WifiClientWidget()
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Click \"Read\" to read current network settings.\n\n"
    "2. Add the settings for your network to the list.\n\n"
    "3. Click \"Write\" to reflect the changes.\n\n",
    kBodyPSize);

  read_button_ = new QPushButton("Read");
  read_button_->setFixedSize(kButtonWidth, kButtonHeight);
  read_button_->setEnabled(true);

  write_button_ = new QPushButton("Write");
  write_button_->setFixedSize(kButtonWidth, kButtonHeight);
  write_button_->setEnabled(false);

  add_button_ = new QPushButton("Add");
  add_button_->setFixedSize(kButtonWidth, kButtonHeight);
  add_button_->setEnabled(false);

  remove_button_ = new QPushButton("Remove");
  remove_button_->setFixedSize(kButtonWidth, kButtonHeight);
  remove_button_->setEnabled(false);

  table_ = new qt::TableWidget(0, kNumCols);
  table_->setHorizontalHeaderLabels({ "SSID", "PSK" });
  table_->setColumnsWidth(kColWidth);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(read_button_);
  cols->addWidget(write_button_);
  cols->addWidget(add_button_);
  cols->addWidget(remove_button_);
  cols->addStretch();

  rows_->addWidget(instruction);
  rows_->addLayout(cols);
  rows_->addWidget(table_);

  // Connection
  connect(read_button_, &QPushButton::clicked, this, &self::onReadButtonClicked);
  connect(write_button_, &QPushButton::clicked, this, &self::onWriteButtonClicked);
  connect(add_button_, &QPushButton::clicked, this, &self::onAddButtonClicked);
  connect(remove_button_, &QPushButton::clicked, this, &self::onRemoveButtonClicked);
}

const char* WifiClientWidget::name() const
{
  return "Network Setting";
}

const char* WifiClientWidget::title() const
{
  return "Configure Wireless Network";
}

void WifiClientWidget::reset()
{
}

void WifiClientWidget::addRow(const std::string& ssid, const std::string& psk)
{
  const auto row = table_->rowCount();
  table_->insertRow(row);
  table_->setItem(row, kSSIDCol, new QTableWidgetItem(QString::fromStdString(ssid)));
  table_->setItem(row, kPSKCol, new QTableWidgetItem(QString::fromStdString(psk)));
}

void WifiClientWidget::onReadButtonClicked()
{
  // TODO
}

void WifiClientWidget::onWriteButtonClicked()
{
  // TODO
}

void WifiClientWidget::onAddButtonClicked()
{
  addRow("", "");
}

void WifiClientWidget::onRemoveButtonClicked()
{
  const auto row = table_->currentRow();
  if (row >= 0) {
    table_->removeRow(row);
  }
}
}  // namespace bm
}  // namespace gui
