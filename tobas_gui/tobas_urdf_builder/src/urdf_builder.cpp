// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder/urdf_builder.hpp"

#include <QAction>
#include <QDockWidget>
#include <QLabel>
#include <QLayout>
#include <QToolButton>
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
  // Initialize the embedded RViz frame with the URDF Builder configuration.
  const auto config_path = getPkgShareDir() / "config/urdf_builder.rviz";
  rviz_manager_.initialize(QString::fromStdString(config_path));

  // RViz creates the editor's dock widget when it loads the configuration above.
  if (const auto panel = rviz_manager_.widget()->findChild<QDockWidget*>("URDF Builder Panel")) {
    // Keep the editor docked by disabling closing, dragging, and floating.
    panel->setFeatures(QDockWidget::NoDockWidgetFeatures);

    // Prevent the dock's menu action from hiding the editor.
    panel->toggleViewAction()->setEnabled(false);

    // Match RViz's marginless render area by removing the editor layout's outer padding.
    panel->widget()->layout()->setContentsMargins(0, 0, 0, 0);

    // RViz's custom title bar must be configured separately from the dock features.
    if (const auto title_bar = panel->titleBarWidget()) {
      // Both the title text and the plugin icon are displayed using labels.
      for (const auto label : title_bar->findChildren<QLabel*>()) {
        label->hide();
      }

      // The custom close button ignores `NoDockWidgetFeatures`, so disable and hide it explicitly.
      for (const auto button : title_bar->findChildren<QToolButton*>()) {
        button->setEnabled(false);
        button->hide();
      }

      // Remove the empty title bar's padding and height so the editor aligns with the RViz render area.
      title_bar->layout()->setContentsMargins(0, 0, 0, 0);
      title_bar->setFixedHeight(0);
    }
  }

  // Embed the RViz frame without adding outer layout margins.
  const auto rows = new QVBoxLayout();
  rows->setContentsMargins(0, 0, 0, 0);
  setLayout(rows);
  rows->addWidget(rviz_manager_.widget());
}
}  // namespace ub
}  // namespace gui
}  // namespace tobas
