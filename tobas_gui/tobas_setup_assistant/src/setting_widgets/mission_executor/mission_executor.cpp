// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/setting_tabs/mission_executor/mission_executor.hpp"

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/controller/custom.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/fixed_wing.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/non_planar_multicopter.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/planar_multicopter.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/random_axis_tilt_multicopter.hpp"
#include "tobas_setup_assistant/setting_tabs/controller/y_axis_tilt_multicopter.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace mission
{
MissionExecutorWidget::MissionExecutorWidget()
{
  stack_ = new qt::StackedWidget();

  none_ = new NoneWidget();
  multicopter_ = new MulticopterWidget();

  stack_->addWidget(none_);
  stack_->addWidget(multicopter_);

  addWidget(stack_);
  addStretch();
}

const char* MissionExecutorWidget::name() const
{
  return "Mission Executor";
}

const char* MissionExecutorWidget::title() const
{
  return "Configure Mission Executor";
}

const char* MissionExecutorWidget::description() const
{
  return "Set the default values for the parameters used when executing a mission. "
         "If no parameters are specified in the Mission Planner, the values configured here will be used. "
         "If parameters are specified in the Mission Planner, they will override the values configured here.";
}

void MissionExecutorWidget::updateInternalDataStructures()
{
}

bool MissionExecutorWidget::isValid()
{
  if (!selected()->isValid()) {
    return false;
  }

  return true;
}

YAML::Node MissionExecutorWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[kNoneKey] = none_->dump();
  node[kMulticopterKey] = multicopter_->dump();

  return node;
}

void MissionExecutorWidget::load(const YAML::Node& node)
{
  none_->load(node[kNoneKey]);
  multicopter_->load(node[kMulticopterKey]);
}

void MissionExecutorWidget::setFrameType(const FrameType& type)
{
  if (
    type == FrameType::kPlanarMulticopter || type == FrameType::kNonPlanarMulticopter ||
    type == FrameType::kYAxisTiltMulticopter || type == FrameType::kRandomAxisTiltMulticopter) {
    stack_->setCurrentWidget(multicopter_);
  }
  else {
    qt::qWarnBox(this, "The Mission Planner does not support this frame type.");
    stack_->setCurrentWidget(none_);
  }
}

QString MissionExecutorWidget::executorPackage() const
{
  return selected()->executorPackage();
}

QString MissionExecutorWidget::pluginName() const
{
  return selected()->pluginName();
}

YAML::Node MissionExecutorWidget::staticParams() const
{
  return selected()->staticParams();
}

BaseExecutorWidget* MissionExecutorWidget::widget(int index)
{
  return qt::qPointerCast<BaseExecutorWidget>(stack_->widget(index));
}

const BaseExecutorWidget* MissionExecutorWidget::widget(int index) const
{
  return qt::qConstPointerCast<BaseExecutorWidget>(stack_->widget(index));
}

BaseExecutorWidget* MissionExecutorWidget::selected()
{
  return qt::qPointerCast<BaseExecutorWidget>(stack_->currentWidget());
}

const BaseExecutorWidget* MissionExecutorWidget::selected() const
{
  return qt::qConstPointerCast<BaseExecutorWidget>(stack_->currentWidget());
}
}  // namespace mission
}  // namespace sa
}  // namespace gui
}  // namespace tobas
