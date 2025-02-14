#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_hardware_setup/network_setting/widget.hpp"
#include "tobas_hardware_setup/constants.hpp"

namespace gui
{
namespace hw
{
NetworkSettingWidget::NetworkSettingWidget(rclcpp::Node::SharedPtr node)
  : spinner_(Qt::WindowModal, this), read_thread_(node), write_thread_(node)
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Press \"Read\" button to read current network settings.\n\n"
    "2. Add the settings for your network to the list.\n\n"
    "3. Press \"Write\" button to reflect the changes .\n\n",
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
  rows_->addStretch();

  // Connection
  connect(read_button_, &QPushButton::clicked, this, &self::onReadButtonClicked);
  connect(write_button_, &QPushButton::clicked, this, &self::onWriteButtonClicked);
  connect(add_button_, &QPushButton::clicked, this, &self::onAddButtonClicked);
  connect(remove_button_, &QPushButton::clicked, this, &self::onRemoveButtonClicked);
  connect(&read_thread_, &ReadWPASupplicantThread::finished, this, &self::onReadThreadFinished);
  connect(&write_thread_, &WriteWPASupplicantThread::finished, this, &self::onWriteThreadFinished);
}

const char* NetworkSettingWidget::name() const
{
  return "Network Setting";
}

const char* NetworkSettingWidget::title() const
{
  return "Configure Wireless Network";
}

void NetworkSettingWidget::reset()
{
}

void NetworkSettingWidget::addRow(const std::string& ssid, const std::string& psk)
{
  const auto row = table_->rowCount();
  table_->insertRow(row);
  table_->setItem(row, kSSIDCol, new QTableWidgetItem(QString::fromStdString(ssid)));
  table_->setItem(row, kPSKCol, new QTableWidgetItem(QString::fromStdString(psk)));
}

void NetworkSettingWidget::onReadButtonClicked()
{
  // 読み取り開始
  read_thread_.start();

  spinner_.show();
  spinner_.start();
}

void NetworkSettingWidget::onWriteButtonClicked()
{
  // WPA Parserにテーブルの内容を反映
  wpa_parser_.networks.clear();
  for (int row = 0; row < table_->rowCount(); ++row)
  {
    wpa_parser_.networks.emplace_back();
    wpa_parser_.networks.back().ssid = table_->item(row, kSSIDCol)->text().toStdString();
    wpa_parser_.networks.back().psk = table_->item(row, kPSKCol)->text().toStdString();
  }

  // 書き込み開始
  write_thread_.setText(wpa_parser_.text());
  write_thread_.start();

  spinner_.show();
  spinner_.start();
}

void NetworkSettingWidget::onAddButtonClicked()
{
  addRow("", "");
}

void NetworkSettingWidget::onRemoveButtonClicked()
{
  const auto row = table_->currentRow();
  if (row >= 0)
    table_->removeRow(row);
}

void NetworkSettingWidget::onReadThreadFinished(bool success, const QString& message)
{
  spinner_.hide();
  spinner_.stop();

  if (!success)
  {
    qt::qErrorBox(this, message);
    return;
  }

  // テキストを解析
  if (!wpa_parser_.parseFromText(read_thread_.getText()))
  {
    qt::qErrorBox(this, "Failed to parse network configuration.");
    return;
  }

  // 現在の設定をテーブルに反映
  table_->removeAll();
  for (const auto& network : wpa_parser_.networks)
    addRow(network.ssid, network.psk);

  // 編集用ボタンを有効化
  write_button_->setEnabled(true);
  add_button_->setEnabled(true);
  remove_button_->setEnabled(true);

  qt::qInfoBox(this, "Network configuration is read successfully.");
}

void NetworkSettingWidget::onWriteThreadFinished(bool success, const QString& message)
{
  spinner_.hide();
  spinner_.stop();

  if (!success)
  {
    qt::qErrorBox(this, message);
    return;
  }

  qt::qInfoBox(this, "Network configuration is written successfully.");
}
}  // namespace hw
}  // namespace gui
