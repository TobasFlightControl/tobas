// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sim
{
class WorldWidget_Standard : public BaseWorldWidget
{
  Q_OBJECT

  using self = WorldWidget_Standard;
  using super = BaseWorldWidget;

public:
  explicit WorldWidget_Standard();

  std::filesystem::path worldPath() const override;

private:
  qt::ComboBox* combo_box_;

  static std::filesystem::path worldDirectoryPath();
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
