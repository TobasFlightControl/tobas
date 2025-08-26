#include "tobas_setup_assistant/setting_tabs/remote_connection/host/ipv4.hpp"

#include <QHBoxLayout>

#include <tobas_qt_tools/message.hpp>

namespace gui
{
namespace sa
{
namespace rc
{
IPv4Widget::IPv4Widget()
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  ipv4_ = new qt::IPv4Edit();
  cols->addWidget(ipv4_);
}

const char* IPv4Widget::label() const
{
  return "IPv4 (DEC)";
}

bool IPv4Widget::isValid()
{
  if (!ipv4_->isFilled()) {
    qt::qWarnBox(this, "Please enter the flight controller’s IP address.");
    return false;
  }

  return true;
}

YAML::Node IPv4Widget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[kAddressKey] = ipv4_->toInt();

  return node;
}

void IPv4Widget::load(const YAML::Node& node)
{
  const auto address = node[kAddressKey].as<uint32_t>();
  ipv4_->setFromInt(address);
}

QString IPv4Widget::host() const
{
  return ipv4_->toString();
}
}  // namespace rc
}  // namespace sa
}  // namespace gui
