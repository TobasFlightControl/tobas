// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QTimer>

#include <tobas_linux/command_executor.hpp>
#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_udev/core.hpp>

#include "./bootmedia.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
class MediaManagerWidget : public QWidget
{
  Q_OBJECT

  using self = MediaManagerWidget;
  using super = QWidget;

Q_SIGNALS:
  void connected(const BootMedia& media);
  void disconnected();

public:
  explicit MediaManagerWidget();

  bool isConnected() const;

private:
  std::unordered_map<QString, BootMedia> medias_;
  linux::CommandExecutor cmd_exec_;

  qt::ComboBox* media_name_;
  qt::ToggleButton* connect_btn_;

  QTimer scan_timer_;

  const BootMedia& currentBootMedia() const;

  static std::pair<std::string, std::string> getVendorAndModel(udev_device* dev);

private Q_SLOTS:
  void onConnectRequested();
  void onDisconnectRequested();
  void onScanTimerTimeout();
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
