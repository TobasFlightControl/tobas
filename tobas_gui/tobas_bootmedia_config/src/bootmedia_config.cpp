#include "tobas_bootmedia_config/bootmedia_config.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>

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

  hostname_ = new HostnameWidget();
  wifi_client_ = new WifiClientWidget();
  wifi_hotspot_ = new WifiHotspotWidget();

  tabs_->addTab(hostname_, hostname_->name());
  tabs_->addTab(wifi_client_, wifi_client_->name());
  tabs_->addTab(wifi_hotspot_, wifi_hotspot_->name());

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

void BootmediaConfigWidget::setTabsEnabled(bool enabled)
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = qt::qPointerCast<BaseConfigWidget>(tabs_->widget(i));
    widget->setEnabled(enabled);
  }
}

void BootmediaConfigWidget::onMediaConnected()
{
  setTabsEnabled(true);
}

void BootmediaConfigWidget::onMediaDisconnected()
{
  reset();
  setTabsEnabled(false);
}
}  // namespace bm
}  // namespace gui
