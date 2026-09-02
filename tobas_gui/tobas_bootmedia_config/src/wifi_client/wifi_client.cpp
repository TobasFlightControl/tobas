// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/wifi_client/wifi_client.hpp"

#include <QDebug>
#include <QEvent>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/password_edit.hpp>
#include <tobas_string_tools/stream.hpp>

#include "tobas_bootmedia_config/constants.hpp"
#include "tobas_bootmedia_config/wifi_client/add_wifi_dialog.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
namespace
{
constexpr int kKeyMgmtCol = 0;
constexpr int kSsidCol = kKeyMgmtCol + 1;
constexpr int kPskCol = kSsidCol + 1;
constexpr int kPriorityCol = kPskCol + 1;
constexpr int kHiddenCol = kPriorityCol + 1;
constexpr int kNumCols = kHiddenCol + 1;

std::string configPath()
{
  return std::string(kRootPath) + "/etc/wpa_supplicant/wpa_supplicant-nl80211-wlan0.conf";
}
}  // namespace

WifiClientWidget::WifiClientWidget()
{
  add_button_ = new QPushButton("Add");
  remove_button_ = new QPushButton("Remove");
  clear_button_ = new QPushButton("Clear");

  add_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);
  remove_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);
  clear_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);

  table_ = new qt::TableWidget(0, kNumCols);
  table_->setHorizontalHeaderLabels({ "AKM", "SSID", "PSK", "Priority", "Hidden" });
  table_->setColumnsWidth(150);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);    // Disable editing.
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);   // Select by row.
  table_->setSelectionMode(QAbstractItemView::SingleSelection);  // Select only one row.
  table_->setHeaderSectionsClickable(false);                     // Disable header clicks.

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(add_button_);
  cols->addWidget(remove_button_);
  cols->addWidget(clear_button_);
  cols->addStretch();

  rows_->addLayout(cols);
  rows_->addWidget(table_);

  // Connection
  connect(add_button_, &QPushButton::clicked, this, &self::onAddButtonClicked);
  connect(remove_button_, &QPushButton::clicked, this, &self::onRemoveButtonClicked);
  connect(clear_button_, &QPushButton::clicked, this, &self::onClearButtonClicked);
  // Immediately after a cell is removed, `QTableWidget::rowCount()` does not yet reflect the updated row count,
  // so use `Qt::QueuedConnection` instead of `Qt::DirectConnection`.
  connect(table_, &QTableWidget::itemSelectionChanged, this, &self::updateTableCellStyles, Qt::QueuedConnection);
}

const char* WifiClientWidget::title() const
{
  return "Configure Wi-Fi Client";
}

void WifiClientWidget::reset()
{
  add_button_->setEnabled(false);
  remove_button_->setEnabled(false);
  clear_button_->setEnabled(false);

  table_->removeAll();
}

bool WifiClientWidget::onConnected()
{
  // Load the configuration file.
  std::string text;
  if (!str::readText(configPath(), text)) {
    qt::qErrorBox(this, "Failed to read network configuration file.");
    return false;
  }

  // Parse the configuration file.
  if (!wpa_parser_.parseFromText(text, wpa_data_)) {
    qt::qErrorBox(this, "Failed to parse network configuration.");
    return false;
  }

  // Reflect the current configuration in the table.
  table_->removeAll();
  for (const auto& network : wpa_data_.networks) {
    addRow(
      QString::fromStdString(wpa::labelFromEnum(network.key_mgmt)),
      QString::fromStdString(network.ssid),
      QString::fromStdString(network.psk),
      network.priority,
      network.scan_ssid);
  }

  // Enable edit buttons.
  add_button_->setEnabled(true);
  remove_button_->setEnabled(true);
  clear_button_->setEnabled(true);

  return true;
}

void WifiClientWidget::changeEvent(QEvent* event)
{
  super::changeEvent(event);

  // Apply the enabled-state palette to each cell.
  if (event->type() == QEvent::EnabledChange && isEnabled()) {
    updateTableCellStyles();
  }
}

QString WifiClientWidget::getKeyMgmt(int row) const
{
  return table_->item(row, kKeyMgmtCol)->text();
}

QString WifiClientWidget::getSsid(int row) const
{
  return table_->item(row, kSsidCol)->text();
}

QString WifiClientWidget::getPsk(int row) const
{
  const auto psk_edit = qt::qConstPointerCast<qt::PasswordEdit>(table_->cellWidget(row, kPskCol));
  return psk_edit->text();
}

int WifiClientWidget::getPriority(int row) const
{
  return table_->item(row, kPriorityCol)->data(Qt::DisplayRole).toInt();
}

bool WifiClientWidget::getHidden(int row) const
{
  return table_->item(row, kHiddenCol)->data(Qt::UserRole).toBool();
}

void WifiClientWidget::addRow(const QString& key_mgmt, const QString& ssid, const QString& psk, int priority, bool hidden)
{
  const auto row = table_->rowCount();
  table_->insertRow(row);

  table_->setItem(row, kKeyMgmtCol, new QTableWidgetItem(key_mgmt));

  table_->setItem(row, kSsidCol, new QTableWidgetItem(ssid));

  const auto psk_edit = new qt::PasswordEdit();
  psk_edit->setText(psk);
  psk_edit->setReadOnly(true);
  psk_edit->setFrame(false);  // Blend in with surrounding cells.
  table_->setCellWidget(row, kPskCol, psk_edit);

  const auto priority_it = new QTableWidgetItem();
  priority_it->setData(Qt::DisplayRole, priority);
  table_->setItem(row, kPriorityCol, priority_it);

  const auto hidden_it = new QTableWidgetItem();
  hidden_it->setData(Qt::UserRole, hidden);
  hidden_it->setData(Qt::DisplayRole, hidden ? "Yes" : "No");
  table_->setItem(row, kHiddenCol, hidden_it);

  if (isEnabled()) {
    updateTableCellStyles();
  }
}

void WifiClientWidget::updateTableCellStyles()
{
  const auto table_palette = table_->palette();

  for (int row = 0; row < table_->rowCount(); ++row) {
    const auto selected = table_->item(row, kSsidCol)->isSelected();
    const auto background = table_palette.color(selected ? QPalette::Highlight : QPalette::Base);
    const auto foreground = table_palette.color(selected ? QPalette::HighlightedText : QPalette::Text);

    const auto qss =
      QString("border: none; background-color: %1; color: %2; selection-background-color: %1; selection-color: %2;")
        .arg(background.name(), foreground.name());

    table_->cellWidget(row, kPskCol)->setStyleSheet(qss);
  }
}

bool WifiClientWidget::writeCurrentConfig()
{
  // Apply the table contents.
  wpa_data_.networks.clear();
  for (int row = 0; row < table_->rowCount(); ++row) {
    wpa::Network network;

    const auto key_mgmt = getKeyMgmt(row);
    if (!wpa::enumFromLabel(key_mgmt.toStdString(), network.key_mgmt)) {
      qt::qErrorBox(this, "Invalid key management: " + key_mgmt);
      return false;
    }

    network.ssid = getSsid(row).toStdString();
    network.psk = getPsk(row).toStdString();
    network.priority = getPriority(row);
    network.scan_ssid = getHidden(row);

    wpa_data_.networks.push_back(network);
  }

  // Write the configuration.
  const auto path = configPath();
  const auto text = wpa_exporter_.exportText(wpa_data_);
  if (!str::writeText(path, text)) {
    qt::qErrorBox(this, "Failed to write to " + QString::fromStdString(path));
    return false;
  }

  qt::qInfoBox(this, "Network configuration is updated successfully.");
  return true;
}

void WifiClientWidget::onAddButtonClicked()
{
  // Get a network from the dialog.
  AddWifiDialog dialog(this);
  const auto result = dialog.exec();
  if (result != QDialog::Accepted) {
    return;
  }

  // Add the network to the table.
  addRow(dialog.getKeyMgmt(), dialog.getSsid(), dialog.getPsk(), dialog.getPriority(), dialog.getHidden());

  // Apply the current configuration to the media.
  if (!writeCurrentConfig()) {
    reset();
    return;
  }
}

void WifiClientWidget::onRemoveButtonClicked()
{
  // Get the rows to delete.
  const auto row = table_->currentRow();
  if (row < 0) {
    qt::qWarnBox(this, "Please select the network to remove.");
    return;
  }

  // Confirm before deleting the selected network.
  if (!qt::yesOrNo(this, "Are you sure you want to remove '" + getSsid(row) + "'?", qt::WARN)) {
    return;
  }

  // Delete the network from the table.
  table_->removeRow(row);

  // Apply the current configuration to the media.
  if (!writeCurrentConfig()) {
    reset();
    return;
  }
}

void WifiClientWidget::onClearButtonClicked()
{
  // Confirm before deleting everything.
  if (!qt::yesOrNo(this, "Are you sure you want to remove all networks?", qt::WARN)) {
    return;
  }

  // Delete the network from the table.
  table_->removeAll();

  // Apply the current configuration to the media.
  if (!writeCurrentConfig()) {
    reset();
    return;
  }
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
