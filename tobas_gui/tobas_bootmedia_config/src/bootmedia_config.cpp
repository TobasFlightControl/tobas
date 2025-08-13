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

  wifi_client_ = new WifiClientWidget();

  tabs_->addTab(wifi_client_, wifi_client_->name());

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

  qt::qInfoBox(this, "The boot media has been mounted successfully.");
}

void BootmediaConfigWidget::onMediaDisconnected()
{
  reset();
  setTabsEnabled(false);

  qt::qInfoBox(this, "The boot media has been unmounted successfully.");
}
}  // namespace bm
}  // namespace gui
