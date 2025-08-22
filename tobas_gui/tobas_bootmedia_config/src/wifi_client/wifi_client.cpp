#include "tobas_bootmedia_config/wifi_client/wifi_client.hpp"

#include <QDebug>

#include <tobas_qt_tools/message.hpp>
#include <tobas_string_tools/stream.hpp>

#include "tobas_bootmedia_config/constants.hpp"
#include "tobas_bootmedia_config/wifi_client/add_wifi_dialog.hpp"

namespace gui
{
namespace bm
{
WifiClientWidget::WifiClientWidget()
{
  read_button_ = new QPushButton("Read");
  add_button_ = new QPushButton("Add");
  remove_button_ = new QPushButton("Remove");
  clear_button_ = new QPushButton("Clear");

  read_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);
  add_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);
  remove_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);
  clear_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);

  table_ = new qt::TableWidget(0, kNumCols);
  table_->setHorizontalHeaderLabels({ "SSID", "PSK", "Priority" });
  table_->setColumnsWidth(kColWidth);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);    // 編集禁止
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);   // 行単位で選択
  table_->setSelectionMode(QAbstractItemView::SingleSelection);  // 1行だけ選択
  table_->setHeaderSectionsClickable(false);                     // ヘッダのクリック禁止

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(read_button_);
  cols->addWidget(add_button_);
  cols->addWidget(remove_button_);
  cols->addWidget(clear_button_);
  cols->addStretch();

  rows_->addLayout(cols);
  rows_->addWidget(table_);

  // Connection
  connect(read_button_, &QPushButton::clicked, this, &self::onReadButtonClicked);
  connect(add_button_, &QPushButton::clicked, this, &self::onAddButtonClicked);
  connect(remove_button_, &QPushButton::clicked, this, &self::onRemoveButtonClicked);
  connect(clear_button_, &QPushButton::clicked, this, &self::onClearButtonClicked);
}

const char* WifiClientWidget::name() const
{
  return "Wi-Fi Client";
}

const char* WifiClientWidget::title() const
{
  return "Configure Wi-Fi Client";
}

void WifiClientWidget::reset()
{
  read_button_->setEnabled(true);
  add_button_->setEnabled(false);
  remove_button_->setEnabled(false);
  clear_button_->setEnabled(false);

  table_->removeAll();
}

QString WifiClientWidget::getSsid(int row) const
{
  return table_->item(row, kSsidCol)->text();
}

QString WifiClientWidget::getPsk(int row) const
{
  return table_->item(row, kPskCol)->data(Qt::UserRole).toString();
}

int WifiClientWidget::getPriority(int row) const
{
  return table_->item(row, kPriorityCol)->data(Qt::DisplayRole).toInt();
}

void WifiClientWidget::addRow(const QString& ssid, const QString& psk, int priority)
{
  const auto row = table_->rowCount();
  table_->insertRow(row);

  table_->setItem(row, kSsidCol, new QTableWidgetItem(ssid));

  const auto psk_it = new QTableWidgetItem();
  psk_it->setData(Qt::UserRole, psk);  // 平文をUserRoleで保持 (EditRoleはDisplayRoleとリンクしているため使えない)
  psk_it->setData(Qt::DisplayRole, QString(psk.size(), QChar(0x25CF)));  // 黒丸で表示
  table_->setItem(row, kPskCol, psk_it);

  const auto priority_it = new QTableWidgetItem();
  priority_it->setData(Qt::DisplayRole, priority);
  table_->setItem(row, kPriorityCol, priority_it);
}

bool WifiClientWidget::writeCurrentConfig()
{
  // テーブルの内容を反映
  wpa_parser_.networks.clear();
  for (int row = 0; row < table_->rowCount(); ++row) {
    wpa_parser_.networks.emplace_back();
    wpa_parser_.networks.back().ssid = getSsid(row).toStdString();
    wpa_parser_.networks.back().psk = getPsk(row).toStdString();
    wpa_parser_.networks.back().priority = getPriority(row);
  }

  // 設定を書き込む
  if (!str::writeText(configPath(), wpa_parser_.exportText())) {
    qt::qErrorBox(this, "Failed to write network configuration.");
    return false;
  }

  return true;
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
    addRow(QString::fromStdString(network.ssid), QString::fromStdString(network.psk), network.priority);
  }

  // 編集用ボタンを有効化
  add_button_->setEnabled(true);
  remove_button_->setEnabled(true);
  clear_button_->setEnabled(true);

  qt::qInfoBox(this, "Network configuration is read successfully.");
}

void WifiClientWidget::onAddButtonClicked()
{
  // ダイアログでネットワークを取得
  AddWifiDialog dialog(this);
  const auto result = dialog.exec();
  if (result != QDialog::Accepted) {
    return;
  }

  // テーブルにネットワークを追加
  addRow(dialog.getSsid(), dialog.getPsk(), dialog.getPriority());

  // 現在の設定をメディアに反映
  if (!writeCurrentConfig()) {
    reset();
    return;
  }
}

void WifiClientWidget::onRemoveButtonClicked()
{
  // 消すべき行を取得
  const auto row = table_->currentRow();
  if (row < 0) {
    qt::qWarnBox(this, "Please select the network to remove.");
    return;
  }

  // 本当に選択したネットワークを消して大丈夫か確認
  if (!qt::yesOrNo(this, "Are you sure you want to remove \"" + getSsid(row) + "\"?", qt::WARN)) {
    return;
  }

  // ネットワークをテーブルから削除
  table_->removeRow(row);

  // 現在の設定をメディアに反映
  if (!writeCurrentConfig()) {
    reset();
    return;
  }
}

void WifiClientWidget::onClearButtonClicked()
{
  // 本当に全削除して大丈夫か確認
  if (!qt::yesOrNo(this, "Are you sure you want to remove all networks?", qt::WARN)) {
    return;
  }

  // ネットワークをテーブルから削除
  table_->removeAll();

  // 現在の設定をメディアに反映
  if (!writeCurrentConfig()) {
    reset();
    return;
  }
}
}  // namespace bm
}  // namespace gui
