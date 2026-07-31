// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/remote_connection/network_iface.hpp"

#include <ranges>

#include <QDebug>
#include <QHBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace rc
{
NetworkIfaceWidget::NetworkIfaceWidget()
{
  nic_btn_group_ = new QButtonGroup(this);
  nic_btn_group_->setExclusive(true);

  int id = 0;
  const auto wired_btn = addNicTypeButton("Wired (Ethernet)", id++);
  const auto wireless_btn = addNicTypeButton("Wireless (Wi-Fi Client)", id++);
  const auto ap_btn = addNicTypeButton("Access Point (Wi-Fi Hotspot)", id++);
  const auto other_btn = addNicTypeButton("Other", id++);

  other_nic_name_ = new QLineEdit();
  other_nic_name_->setPlaceholderText("e.g. wwan0, eth1, enx...");
  other_nic_name_->setEnabled(false);

  // Layout
  const auto other_row = new QHBoxLayout();
  other_row->addWidget(other_btn);
  other_row->addWidget(other_nic_name_);

  const auto rows = new QVBoxLayout();
  rows->addWidget(wired_btn);
  rows->addWidget(wireless_btn);
  rows->addWidget(ap_btn);
  rows->addLayout(other_row);

  setLayout(rows);

  // Connection
  connect(other_btn, &QRadioButton::toggled, this, &self::onOtherButtonToggled);
}

void NetworkIfaceWidget::setToDefaults()
{
  nic_btn_group_->button(kWirelessIdx)->setChecked(true);
  other_nic_name_->clear();
}

bool NetworkIfaceWidget::isValid()
{
  if (nic_btn_group_->checkedId() == kOtherIdx) {
    if (other_nic_name_->text().isEmpty()) {
      qt::qWarnBox(this, "Please specify a network interface name.");
      return false;
    }
  }

  return true;
}

YAML::Node NetworkIfaceWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  const auto cur_btn = nic_btn_group_->checkedButton();
  node[kNicTypeKey] = cur_btn->text();

  node[kOtherNicNameKey] = other_nic_name_->text();

  return node;
}

void NetworkIfaceWidget::load(const YAML::Node& node)
{
  const auto nic_type_text = node[kNicTypeKey].as<QString>();
  for (const auto& [idx, btn] : std::views::enumerate(nic_btn_group_->buttons())) {
    if (btn->text() == nic_type_text) {
      nic_btn_group_->button(idx)->setChecked(true);
      break;
    }
  }

  other_nic_name_->setText(node[kOtherNicNameKey].as<QString>());
}

QString NetworkIfaceWidget::networkInterface() const
{
  const auto id = nic_btn_group_->checkedId();
  switch (id) {
    case kWiredIdx:
      return "eth0";
    case kWirelessIdx:
      return "wlan0";
    case kAccessPointIdx:
      return "ap0";
    case kOtherIdx:
      return other_nic_name_->text();
    default:
      throw std::runtime_error("Invalid network interface ID: " + std::to_string(id));
  }
}

QRadioButton* NetworkIfaceWidget::addNicTypeButton(const QString& text, int id)
{
  const auto btn = new QRadioButton(text);
  nic_btn_group_->addButton(btn, id);
  return btn;
}

void NetworkIfaceWidget::onOtherButtonToggled(bool checked)
{
  other_nic_name_->setEnabled(checked);

  if (checked) {
    other_nic_name_->setFocus();
    other_nic_name_->selectAll();
  }
}
}  // namespace rc
}  // namespace sa
}  // namespace gui
}  // namespace tobas
