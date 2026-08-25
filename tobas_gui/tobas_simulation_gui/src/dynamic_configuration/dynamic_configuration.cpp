// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/dynamic_configuration/dynamic_configuration.hpp"

#include <QVBoxLayout>

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace ch = std::chrono;

namespace tobas
{
namespace gui
{
namespace sim
{
DynamicConfigWidget::DynamicConfigWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto title = new qt::Label("Dynamic Configurations", cmn::kTitlePSize, QFont::Bold);
  qt::addWidgetCenter(title, rows);

  const auto scroll_rows = qt::createScrollableQVBoxLayout(rows);

  wind_params_ = new WindParamsWidget();
  scroll_rows->addWidget(wind_params_);

  suspended_load_ = new SuspendedLoadWidget();
  scroll_rows->addWidget(suspended_load_);

  scroll_rows->addStretch();
}

void DynamicConfigWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  wind_params_->initializeRosInterfaces(node, ns);
  suspended_load_->initializeRosInterfaces(node, ns);
}

void DynamicConfigWidget::clearRosInterfaces()
{
  wind_params_->clearRosInterfaces();
  suspended_load_->clearRosInterfaces();
}

bool DynamicConfigWidget::start(ch::milliseconds timeout)
{
  if (!wind_params_->start(timeout)) {
    return false;
  }

  if (!suspended_load_->start(timeout)) {
    return false;
  }

  return true;
}

void DynamicConfigWidget::reset()
{
  wind_params_->reset();
  suspended_load_->reset();
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
