// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/bootmedia_config.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/font.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_bootmedia_config/hostname/hostname.hpp"
#include "tobas_bootmedia_config/ip_address/ip_address.hpp"
#include "tobas_bootmedia_config/login_password/login_password.hpp"
#include "tobas_bootmedia_config/ssh_authorized_keys/ssh_authorized_keys.hpp"
#include "tobas_bootmedia_config/wifi_client/wifi_client.hpp"
#include "tobas_bootmedia_config/wifi_hotspot/wifi_hotspot.hpp"

namespace tobas::gui::bm
{
BootmediaConfigWidget::BootmediaConfigWidget()
{
  media_manager_ = new MediaManagerWidget();

  tabs_ = new qt::VerticalTabWidget();
  tabs_->enableWheelEvent(false);
  tabs_->setTabSize(64, 32);

  tabs_->addTab(createTabPage(new HostnameWidget()), "Hostname");
  tabs_->addTab(createTabPage(new LoginPasswordWidget()), "Login Password");
  tabs_->addTab(createTabPage(new SshAuthorizedKeysWidget()), "SSH Keys");
  tabs_->addTab(createTabPage(new WifiClientWidget()), "Wi-Fi Client");
  tabs_->addTab(createTabPage(new WifiHotspotWidget()), "Wi-Fi Hotspot");
  tabs_->addTab(createTabPage(new IpAddressWidget()), "IP Address");

  reset();
  tabs_->setTabWidgetsEnabled(false);

  // Layout
  const auto header_cols = new QHBoxLayout();
  header_cols->addStretch(1);
  header_cols->addWidget(media_manager_);

  const auto root_rows = new QVBoxLayout(this);
  root_rows->addLayout(header_cols);
  root_rows->addWidget(tabs_, 1);

  // Connection
  connect(media_manager_, &MediaManagerWidget::connected, this, &self::onMediaConnected);
  connect(media_manager_, &MediaManagerWidget::disconnected, this, &self::onMediaDisconnected);
}

void BootmediaConfigWidget::reset()
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = getWidget(i);
    widget->reset();
  }
}

void BootmediaConfigWidget::closeEvent(QCloseEvent* _event)
{
  if (media_manager_->isConnected()) {
    if (!qt::yesOrNo(
          this,
          "You’re attempting to close the application while the boot device is still connected. "
          "Are you sure you want to exit?",
          qt::WARN)) {
      _event->ignore();
      return;
    }
  }

  _event->accept();
}

BaseConfigWidget* BootmediaConfigWidget::getWidget(int _index)
{
  return tabs_->widget(_index)->findChild<BaseConfigWidget*>({}, Qt::FindDirectChildrenOnly);
}

QWidget* BootmediaConfigWidget::createTabPage(BaseConfigWidget* _content)
{
  const auto page = new QWidget();
  page->setAutoFillBackground(true);

  const auto title = new QLabel(_content->title());
  title->setFont(qt::DefaultFont(cmn::kTitlePointSize, QFont::Bold));

  _content->layout()->setContentsMargins(0, 0, 0, 0);

  const auto rows = new QVBoxLayout(page);
  rows->addWidget(title, 0, Qt::AlignTop);
  rows->addSpacing(32);
  rows->addWidget(_content, 1);

  return page;
}

void BootmediaConfigWidget::onMediaConnected()
{
  for (int i = 0; i < tabs_->count(); ++i) {
    getWidget(i)->onConnected();
  }

  tabs_->setTabWidgetsEnabled(true);
}

void BootmediaConfigWidget::onMediaDisconnected()
{
  reset();
  tabs_->setTabWidgetsEnabled(false);
}
}  // namespace tobas::gui::bm
