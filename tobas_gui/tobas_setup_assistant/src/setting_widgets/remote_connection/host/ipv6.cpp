// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/remote_connection/host/ipv6.hpp"

#include <QHBoxLayout>

#include <tobas_qt_tools/message.hpp>
#include <tobas_yaml_tools/convert/uint128.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace rc
{
IPv6Widget::IPv6Widget()
{
  const auto cols = new QHBoxLayout();
  setLayout(cols);

  ipv6_ = new qt::IPv6Edit();
  cols->addWidget(ipv6_);
}

void IPv6Widget::setToDefaults()
{
  ipv6_->clear();
}

const char* IPv6Widget::label() const
{
  return "IPv6 (HEX)";
}

bool IPv6Widget::isValid()
{
  if (!ipv6_->isFilled()) {
    qt::qWarnBox(this, "Please enter the flight controller’s IP address.");
    return false;
  }

  return true;
}

YAML::Node IPv6Widget::dump()
{
  YAML::Node node(YAML::NodeType::Map);

  node[kAddressKey] = ipv6_->toInt();

  return node;
}

void IPv6Widget::load(const YAML::Node& node)
{
  const auto address = node[kAddressKey].as<__uint128_t>();
  ipv6_->setFromInt(address);
}

QString IPv6Widget::host() const
{
  return '[' + ipv6_->toString() + ']';  // Add brackets to avoid colon conflicts.
}
}  // namespace rc
}  // namespace sa
}  // namespace gui
}  // namespace tobas
