#include "tobas_bootmedia_config/bootmedia_config.hpp"

#include <QApplication>
#include <QVBoxLayout>

#include <tobas_qt_tools/cast.hpp>

namespace gui
{
namespace bm
{
BootmediaConfigWidget::BootmediaConfigWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  tabs_ = new qt::VerticalTabWidget();
  tabs_->enableWheelEvent(false);
  rows->addWidget(tabs_);

  wifi_sta_ = new WifiStationWidget();

  tabs_->addTab(wifi_sta_, wifi_sta_->name());

  tabs_->setTabSize(kTabWidth, kTabHeight);

  setTabsEnabled(false);
}

void BootmediaConfigWidget::reset()
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = qt::qPointerCast<BaseConfigWidget>(tabs_->widget(i));
    widget->reset();
  }

  tabs_->setCurrentWidget(wifi_sta_);
}

void BootmediaConfigWidget::setTabsEnabled(bool enabled)
{
  wifi_sta_->setEnabled(enabled);
}
}  // namespace bm
}  // namespace gui
