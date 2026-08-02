// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "../base.hpp"
#include "../propeller.hpp"
#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class AerodynamicsWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

  static constexpr char kMethodNameKey[] = "method_name";

public:
  explicit AerodynamicsWidget(const PropellerWidget* propeller);

  void setToDefaults() override;
  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  /* [kg*m/rad^2] */
  double motorConst() const;

  /* [m] */
  double momentConst() const;

  /* [kg/rad] */
  double dragConst() const;

private:
  const PropellerWidget* const propeller_;

  qt::ComboBox* method_name_;
  qt::StackedWidget* methods_;

  AerodynamicsWidget_Base* selected();
  const AerodynamicsWidget_Base* selected() const;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
