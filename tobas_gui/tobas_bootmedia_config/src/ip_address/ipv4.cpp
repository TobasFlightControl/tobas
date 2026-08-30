// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/ip_address/ipv4.hpp"

#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>

namespace tobas
{
namespace gui
{
namespace bm
{
IPv4Widget::IPv4Widget(const QString& name) : name_(name)
{
  const auto form = new qt::FormLayout();
  setLayout(form);

  method_ = new qt::ComboBox();
  method_->addItem("Automatic");
  method_->addItem("Manual");
  form->addVAlignedRow("Method", method_);
  connect(method_, qOverload<int>(&QComboBox::currentIndexChanged), this, &self::onMethodChanged);

  prefix_ = new qt::ComboBox();
  prefix_->addItem("8 - 255.0.0.0");
  prefix_->addItem("16 - 255.255.0.0");
  prefix_->addItem("24 - 255.255.255.0");
  prefix_->addItem("25 - 255.255.255.128");
  prefix_->addItem("26 - 255.255.255.192");
  prefix_->addItem("27 - 255.255.255.224");
  prefix_->addItem("28 - 255.255.255.240");
  prefix_->addItem("29 - 255.255.255.248");
  prefix_->addItem("30 - 255.255.255.252");
  prefix_->addItem("32 - 255.255.255.255");
  form->addVAlignedRow("Prefix Length", prefix_);

  address_ = new qt::IPv4Edit();
  form->addVAlignedRow("Address", address_);

  gateway_ = new qt::IPv4Edit();
  form->addVAlignedRow("Gateway", gateway_);
}

QString IPv4Widget::name() const
{
  return name_;
}

void IPv4Widget::reset()
{
  method_->setCurrentIndex(Method::kAutomatic);
  prefix_->setCurrentIndex(PrefixLength::kSlash24);
  address_->clear();
  gateway_->clear();

  enableManualFields(false);
}

bool IPv4Widget::load(const Network& src)
{
  if (src.automatic) {
    method_->setCurrentIndex(Method::kAutomatic);
  }
  else {
    method_->setCurrentIndex(Method::kManual);

    const auto prefix = prefixValueToIndex(src.manual.prefix);
    if (!prefix) {
      qt::qErrorBox(this, prefix.error());
      return false;
    }
    prefix_->setCurrentIndex(*prefix);

    address_->setFromInt(src.manual.address);
    gateway_->setFromInt(src.manual.gateway);
  }

  return true;
}

Network IPv4Widget::dump() const
{
  Network res;

  res.name = name_.toStdString();

  switch (method_->currentIndex()) {
    case Method::kAutomatic:
      res.automatic = true;
      break;
    case Method::kManual:
      res.automatic = false;
      res.manual.address = address_->toInt();
      res.manual.prefix = prefixIndexToValue(static_cast<PrefixLength>(prefix_->currentIndex()));
      res.manual.gateway = gateway_->toInt();
      res.manual.dns.push_back(0x01010101);  // one.one.one.one
      res.manual.dns.push_back(0x08080808);  // dns.google
      break;
    default:
      throw std::runtime_error("Invalid method.");
  }

  return res;
}

void IPv4Widget::enableManualFields(bool enabled)
{
  prefix_->setEnabled(enabled);
  address_->setEnabled(enabled);
  gateway_->setEnabled(enabled);
}

std::expected<IPv4Widget::PrefixLength, QString> IPv4Widget::prefixValueToIndex(uint8_t value)
{
  switch (value) {
    case 8:
      return PrefixLength::kSlash8;
    case 16:
      return PrefixLength::kSlash16;
    case 24:
      return PrefixLength::kSlash24;
    case 25:
      return PrefixLength::kSlash25;
    case 26:
      return PrefixLength::kSlash26;
    case 27:
      return PrefixLength::kSlash27;
    case 28:
      return PrefixLength::kSlash28;
    case 29:
      return PrefixLength::kSlash29;
    case 30:
      return PrefixLength::kSlash30;
    case 32:
      return PrefixLength::kSlash32;
    default:
      return std::unexpected("Unexpected IPv4 prefix length: " + QString::number(value));
  }
}

uint8_t IPv4Widget::prefixIndexToValue(PrefixLength index)
{
  switch (index) {
    case PrefixLength::kSlash8:
      return 8;
    case PrefixLength::kSlash16:
      return 16;
    case PrefixLength::kSlash24:
      return 24;
    case PrefixLength::kSlash25:
      return 25;
    case PrefixLength::kSlash26:
      return 26;
    case PrefixLength::kSlash27:
      return 27;
    case PrefixLength::kSlash28:
      return 28;
    case PrefixLength::kSlash29:
      return 29;
    case PrefixLength::kSlash30:
      return 30;
    case PrefixLength::kSlash32:
      return 32;
    default:
      throw;
  }
}

void IPv4Widget::onMethodChanged(int index)
{
  switch (index) {
    case Method::kAutomatic:
      enableManualFields(false);
      break;
    case Method::kManual:
      enableManualFields(true);
      break;
    default:
      throw std::runtime_error("Invalid method.");
  }
}
}  // namespace bm
}  // namespace gui
}  // namespace tobas
