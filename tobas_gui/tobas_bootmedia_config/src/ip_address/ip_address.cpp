#include "tobas_bootmedia_config/ip_address/ip_address.hpp"

#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>

#include "tobas_bootmedia_config/constants.hpp"
#include "tobas_bootmedia_config/ip_address/ipv4.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
IpAddressWidget::IpAddressWidget()
{
  tabs_ = new qt::TabWidget();
  tabs_->setTabSize(kTabWidth, kTabHeight);
  tabs_->addTab(new IPv4Widget("eth0"), "Wired");
  tabs_->addTab(new IPv4Widget("wlan0"), "Wireless");

  write_button_ = new QPushButton("Write");
  write_button_->setFixedSize(kCtrlButtonWidth, kCtrlButtonHeight);

  // Layout
  rows_->addWidget(tabs_);
  rows_->addSpacing(30);
  qt::addWidgetCenter(write_button_, rows_);
  rows_->addStretch();

  // Connection
  connect(write_button_, &QPushButton::clicked, this, &self::onWriteButtonClicked);
}

const char* IpAddressWidget::title() const
{
  return "Specify IP Address";
}

void IpAddressWidget::reset()
{
  for (int i = 0; i < tabs_->count(); ++i) {
    getWidget(i)->reset();
  }
}

bool IpAddressWidget::onConnected()
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto widget = getWidget(i);
    const auto path = networkFilePath(widget->name().toStdString());

    Network network;
    if (network.load(path)) {
      if (!getWidget(i)->load(network)) {
        return false;
      }
    }
  }

  return true;
}

BaseNetworkWidget* IpAddressWidget::getWidget(int index)
{
  return qt::qPointerCast<BaseNetworkWidget>(tabs_->widget(index));
}

std::string IpAddressWidget::networkFilePath(const std::string& name)
{
  return std::string(kRootPath) + "/etc/systemd/network/20-" + name + ".network";
}

void IpAddressWidget::onWriteButtonClicked()
{
  for (int i = 0; i < tabs_->count(); ++i) {
    const auto network = getWidget(i)->dump();
    const auto path = networkFilePath(network.name);
    if (!network.save(path)) {
      qt::qErrorBox(this, "Failed to write the settings of \"" + QString::fromStdString(network.name) + "\".");
      return;
    }
  }

  qt::qInfoBox(this, "IP address configuration was written successfully.");
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
