// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/network/network.hpp"

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace network
{
namespace
{
constexpr char kNetworkIfaceLabel[] = "Network Interface";
}  // namespace

NetworkWidget::NetworkWidget()
{
  nic_ = new NetworkIfaceWidget();

  addWidget(new qt::Label(kNetworkIfaceLabel, cmn::kLabelPSize, QFont::Bold));
  addWidget(new qt::DescriptionWidget("Specify the network interface used by the flight controller.", cmn::kBodyPSize));
  addWidget(nic_);

  addStretch();
}

const char* NetworkWidget::name() const
{
  return "Network";
}

const char* NetworkWidget::title() const
{
  return "Set up FC Network";
}

const char* NetworkWidget::description() const
{
  return "Configure the settings required for communication between the flight controller and external devices, "
         "such as a ground control station or companion PC.";
}

void NetworkWidget::updateInternalDataStructures()
{
  return;
}

void NetworkWidget::setToDefaults()
{
  nic_->setToDefaults();
}

bool NetworkWidget::isValid()
{
  if (!nic_->isValid()) {
    return false;
  }
  return true;
}

YAML::Node NetworkWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNetworkIfaceLabel] = nic_->dump();

  return node;
}

void NetworkWidget::load(const YAML::Node& node)
{
  nic_->load(node[kNetworkIfaceLabel]);
}

QString NetworkWidget::networkInterface() const
{
  return nic_->networkInterface();
}

}  // namespace network
}  // namespace sa
}  // namespace gui
}  // namespace tobas
