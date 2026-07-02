// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace field
{
class StopAtWaypointWidget : public FieldWidget<bool>
{
public:
  explicit StopAtWaypointWidget();

  const char* label() const override;

  bool getValue() const override;
  void setValue(bool value) override;

private:
  qt::ComboBox* combobox_;
};
}  // namespace field
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
