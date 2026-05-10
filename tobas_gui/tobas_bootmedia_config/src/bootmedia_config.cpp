// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/bootmedia_config.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_bootmedia_config/hostname/hostname.hpp"
#include "tobas_bootmedia_config/login_password/login_password.hpp"
#include "tobas_bootmedia_config/ssh_authorized_keys/ssh_authorized_keys.hpp"
#include "tobas_bootmedia_config/wifi_client/wifi_client.hpp"
#include "tobas_bootmedia_config/wifi_hotspot/wifi_hotspot.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
BootmediaConfigWidget::BootmediaConfigWidget()
{
  media_manager_ = new MediaManagerWidget();

  tabs_ = new qt::VerticalTabWidget();
  tabs_->enableWheelEvent(false);
  tabs_->setTabSize(kTabWidth, kTabHeight);

  tabs_->addTab(new HostnameWidget(), "Hostname");
  tabs_->addTab(new LoginPasswordWidget(), "Login Password");
  tabs_->addTab(new SshAuthorizedKeysWidget(), "SSH Keys");
  tabs_->addTab(new WifiClientWidget(), "Wi-Fi Client");
  tabs_->addTab(new WifiHotspotWidget(), "Wi-Fi Hotspot");

  reset();
  setTabsEnabled(false);

  // Layout
  const auto header_cols = new QHBoxLayout();
  header_cols->addStretch(1);
  header_cols->addWidget(media_manager_);

  const auto root_rows = new QVBoxLayout();
  root_rows->addLayout(header_cols);
  root_rows->addWidget(tabs_);

  setLayout(root_rows);

  // Connection
  connect(media_manager_, &MediaManagerWidget::connected, this, &self::onMediaConnected);
  connect(media_manager_, &MediaManagerWidget::disconnected, this, &self::onMediaDisconnected);
}

void BootmediaConfigWidget::reset()
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = qt::qPointerCast<BaseConfigWidget>(tabs_->widget(i));
    widget->reset();
  }
}

void BootmediaConfigWidget::closeEvent(QCloseEvent* event)
{
  if (media_manager_->isConnected()) {
    if (!qt::yesOrNo(
          this,
          "You’re attempting to close the application while the boot device is still connected. "
          "Are you sure you want to exit?",
          qt::WARN)) {
      event->ignore();
      return;
    }
  }

  event->accept();
}

BaseConfigWidget* BootmediaConfigWidget::getWidget(int index)
{
  return qt::qPointerCast<BaseConfigWidget>(tabs_->widget(index));
}

void BootmediaConfigWidget::setTabsEnabled(bool enabled)
{
  for (int i = 0; i < tabs_->count(); ++i) {
    getWidget(i)->setEnabled(enabled);
  }
}

void BootmediaConfigWidget::onMediaConnected()
{
  for (int i = 0; i < tabs_->count(); ++i) {
    getWidget(i)->onConnected();
  }

  setTabsEnabled(true);
}

void BootmediaConfigWidget::onMediaDisconnected()
{
  reset();
  setTabsEnabled(false);
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
