// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../base_setting.hpp"
#include "./multicopter.hpp"
#include "./none.hpp"
#include "tobas_setup_assistant/frame_type.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace mission
{
class MissionExecutorWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = MissionExecutorWidget;
  using super = BaseSettingWidget;

  static constexpr char kNoneKey[] = "none";
  static constexpr char kMulticopterKey[] = "multicopter";

public:
  explicit MissionExecutorWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  void setToDefaults() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  void setFrameType(const FrameType& type);

  QString executorPackage() const;
  QString pluginName() const;

  YAML::Node staticParams() const;

private:
  qt::StackedWidget* stack_;

  NoneWidget* none_;
  MulticopterWidget* multicopter_;

  BaseExecutorWidget* widget(int index);
  const BaseExecutorWidget* widget(int index) const;

  BaseExecutorWidget* selected();
  const BaseExecutorWidget* selected() const;
};
}  // namespace mission
}  // namespace sa
}  // namespace gui
}  // namespace tobas
