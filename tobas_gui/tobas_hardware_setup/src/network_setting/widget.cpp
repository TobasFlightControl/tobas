#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

#include "tobas_hardware_setup/network_setting/widget.hpp"
#include "tobas_hardware_setup/constants.hpp"

namespace gui
{
namespace hardware_setup
{
NetworkSettingWidget::NetworkSettingWidget(rclcpp::Node::SharedPtr node) : ssh_client_(node)
{
  const auto instruction = new qt::DescriptionWidget(
    "1. Press \"Read\" button to read current network settings.\n\n"
    "2. Add the settings for your network to the list.\n\n"
    "3. Press \"Write\" button to reflect the changes .\n\n",
    kBodyPSize);
  rows_->addWidget(instruction);

  const auto cols = new QHBoxLayout();
  rows_->addLayout(cols);

  read_button_ = new QPushButton("Read");
  read_button_->setFixedSize(kButtonWidth, kButtonHeight);
  read_button_->setEnabled(true);
  connect(read_button_, &QPushButton::clicked, this, &self::onReadButtonClicked);
  cols->addWidget(read_button_);

  write_button_ = new QPushButton("Write");
  write_button_->setFixedSize(kButtonWidth, kButtonHeight);
  write_button_->setEnabled(false);
  connect(write_button_, &QPushButton::clicked, this, &self::onWriteButtonClicked);
  cols->addWidget(write_button_);

  add_button_ = new QPushButton("Add");
  add_button_->setFixedSize(kButtonWidth, kButtonHeight);
  add_button_->setEnabled(false);
  connect(add_button_, &QPushButton::clicked, this, &self::onAddButtonClicked);
  cols->addWidget(add_button_);

  remove_button_ = new QPushButton("Remove");
  remove_button_->setFixedSize(kButtonWidth, kButtonHeight);
  remove_button_->setEnabled(false);
  connect(remove_button_, &QPushButton::clicked, this, &self::onRemoveButtonClicked);
  cols->addWidget(remove_button_);

  cols->addStretch();

  table_ = new qt::TableWidget(0, kNumCols);
  table_->setHorizontalHeaderLabels({ "SSID", "PSK" });
  table_->setColumnsWidth(kColWidth);
  rows_->addWidget(table_);

  rows_->addStretch();
}

const char* NetworkSettingWidget::name() const
{
  return "Network Setting";
}

const char* NetworkSettingWidget::title() const
{
  return "Setup Network";
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
  // SSH接続を確認
  if (ssh_client_.connect() != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, "No SSH connection: " + QString(ssh_client_.errorMessage()));
    return;
  }

  // リモートファイルを開いて内容を読む
  std::string config_text;
  if (ssh_client_.sftpRead(kFilePath, config_text, true) != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, ssh_client_.errorMessage());
    return;
  }

  // 解析の成否に関わらず編集用ボタンを有効化
  write_button_->setEnabled(true);
  add_button_->setEnabled(true);
  remove_button_->setEnabled(true);

  // テキストを解析
  if (!wpa_parser_.parseFromText(config_text))
  {
    qt::qErrorBox(this, "Failed to parse network configuration.");
    return;
  }

  // 現在の設定をテーブルに反映
  table_->removeAll();
  for (const auto& network : wpa_parser_.networks)
    addRow(network.ssid, network.psk);

  qt::qInfoBox(this, "Network configuration is read successfully.");
}

void NetworkSettingWidget::onWriteButtonClicked()
{
  // SSH接続を確認
  if (ssh_client_.connect() != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, "No SSH connection: " + QString(ssh_client_.errorMessage()));
    return;
  }

  // WPA Parserにテーブルの内容を反映
  wpa_parser_.networks.clear();
  for (int row = 0; row < table_->rowCount(); ++row)
  {
    wpa_parser_.networks.emplace_back();
    wpa_parser_.networks.back().ssid = table_->item(row, kSSIDCol)->text().toStdString();
    wpa_parser_.networks.back().psk = table_->item(row, kPSKCol)->text().toStdString();
  }

  // 設定を書き込む
  if (ssh_client_.sftpWrite(kFilePath, wpa_parser_.text(), true) != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, "SFTP-Write failed: " + QString(ssh_client_.errorMessage()));
    return;
  }

  // WiFiを再起動
  if (ssh_client_.execute("wpa_cli -i wlan0 reconfigure", true) != ssh::SSHClient::E_NO_ERROR)
  {
    qt::qErrorBox(this, "Failed to restart DHCPCD: " + QString(ssh_client_.errorMessage()));
    return;
  }

  qt::qInfoBox(this, "Network configuration is written successfully.");
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
}  // namespace hardware_setup
}  // namespace gui
