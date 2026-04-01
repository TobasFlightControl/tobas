// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder/urdf_builder.hpp"

#include <QVBoxLayout>

#include "tobas_urdf_builder/util.hpp"

namespace fs = std::filesystem;

namespace tobas
{
namespace gui
{
namespace ub
{
URDFBuilder::URDFBuilder() : rviz_manager_("rviz_urdf_builder")
{
  const auto config_path = getPkgShareDir() / "config/urdf_builder.rviz";
  rviz_manager_.initialize(QString::fromStdString(config_path));

  const auto rows = new QVBoxLayout();
  setLayout(rows);
  rows->addWidget(rviz_manager_.widget());
}
}  // namespace ub
}  // namespace gui
}  // namespace tobas
