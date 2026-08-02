// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLineEdit>
#include <QPushButton>
#include <QSettings>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sim
{
class CustomWorldWidget : public BaseWorldWidget
{
  Q_OBJECT

  using self = CustomWorldWidget;
  using super = BaseWorldWidget;

  static constexpr char kLastOpenedDirKey[] = "simulation_settings/world/custom_world/last_opened_dir";

public:
  explicit CustomWorldWidget();

  std::filesystem::path worldPath() const override;

private:
  QSettings settings_store_;

  QLineEdit* file_text_;
  QPushButton* browse_button_;

private Q_SLOTS:
  void onBrowseButtonClicked();
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
