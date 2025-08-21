#include "tobas_bootmedia_config/wifi_client/wifi_client.hpp"

#include <tobas_qt_tools/message.hpp>
#include <tobas_string_tools/stream.hpp>

#include "tobas_bootmedia_config/constants.hpp"

namespace gui
{
namespace bm
{
WifiClientWidget::WifiClientWidget()
{
  read_button_ = new QPushButton("Read");
  read_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);

  write_button_ = new QPushButton("Write");
  write_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);

  add_button_ = new QPushButton("Add");
  add_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);

  remove_button_ = new QPushButton("Remove");
  remove_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);

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
  read_button_->setEnabled(true);
  write_button_->setEnabled(false);
  add_button_->setEnabled(false);
  remove_button_->setEnabled(false);

  table_->removeAll();
}

QString WifiClientWidget::getSsid(int row) const
{
  return table_->item(row, kSsidCol)->text();
}

QString WifiClientWidget::getPsk(int row) const
{
  return table_->item(row, kPskCol)->text();
}

void WifiClientWidget::addRow(const std::string& ssid, const std::string& psk)
{
  const auto row = table_->rowCount();
  table_->insertRow(row);
  table_->setItem(row, kSsidCol, new QTableWidgetItem(QString::fromStdString(ssid)));
  table_->setItem(row, kPskCol, new QTableWidgetItem(QString::fromStdString(psk)));
}

std::string WifiClientWidget::configPath()
{
  return std::string(kRootPath) + "/etc/wpa_supplicant/wpa_supplicant.conf";
}

void WifiClientWidget::onReadButtonClicked()
{
  // 設定ファイルを読み込む
  std::string text;
  if (!str::readText(configPath(), text)) {
    qt::qErrorBox(this, "Failed to read network configuration file.");
    return;
  }

  // 設定ファイルを解析
  if (!wpa_parser_.parseFromText(text)) {
    qt::qErrorBox(this, "Failed to parse network configuration.");
    return;
  }

  // 現在の設定をテーブルに反映
  table_->removeAll();
  for (const auto& network : wpa_parser_.networks) {
    addRow(network.ssid, network.psk);
  }

  // 編集用ボタンを有効化
  write_button_->setEnabled(true);
  add_button_->setEnabled(true);
  remove_button_->setEnabled(true);

  qt::qInfoBox(this, "Network configuration is read successfully.");
}

void WifiClientWidget::onWriteButtonClicked()
{
  // テーブルの内容を反映
  wpa_parser_.networks.clear();
  for (int row = 0; row < table_->rowCount(); ++row) {
    wpa_parser_.networks.emplace_back();
    wpa_parser_.networks.back().ssid = getSsid(row).toStdString();
    wpa_parser_.networks.back().psk = getPsk(row).toStdString();
  }

  // 設定を書き込む
  if (!str::writeText(configPath(), wpa_parser_.exportText())) {
    qt::qErrorBox(this, "Failed to write network configuration.");
    return;
  }

  qt::qInfoBox(this, "Network configuration is written successfully.");
}

void WifiClientWidget::onAddButtonClicked()
{
  addRow("", "");
}

void WifiClientWidget::onRemoveButtonClicked()
{
  const auto row = table_->currentRow();
  if (row < 0) {
    qt::qWarnBox(this, "Please select the network to remove.");
    return;
  }

  if (!qt::yesOrNo(this, "Are you sure you want to remove \"" + getSsid(row) + "\"?", qt::WARN)) {
    return;
  }

  table_->removeRow(row);
}
}  // namespace bm
}  // namespace gui
