// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPointer>
#include <QRect>
#include <QSettings>
#include <QTimer>
#include <QWindow>
#include <gz/gui/Plugin.hh>

namespace tobas::gazebo
{
/**
 * Gazebo GUI plugin that saves and restores the main window's position, size, and maximized state.
 *
 * Settings are shared across worlds and saved while the window is running to survive simulation termination.
 * Restored geometry is adjusted to fit the available screens.
 */
class TobasWindowGeometry : public gz::gui::Plugin
{
  Q_OBJECT

public:
  void LoadConfig(const tinyxml2::XMLElement* _elem) override;

protected:
  bool eventFilter(QObject* _obj, QEvent* _event) override;

private:
  void initialize();
  void save();

  QPointer<QWindow> window_;
  QSettings settings_{ QSettings::IniFormat, QSettings::UserScope, "Tobas", "GazeboSim" };
  QTimer save_timer_;
  QRect normal_geometry_;
};
}  // namespace tobas::gazebo
