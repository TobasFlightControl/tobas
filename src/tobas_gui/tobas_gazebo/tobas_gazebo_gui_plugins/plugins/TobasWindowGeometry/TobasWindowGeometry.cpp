// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "./TobasWindowGeometry.hpp"

#include <algorithm>

#include <QCoreApplication>
#include <QDebug>
#include <QEvent>
#include <QGuiApplication>
#include <QScreen>
#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>

namespace tobas::gazebo
{
namespace
{
constexpr char kGeometryKey[] = "geometry";
constexpr char kMaximizedKey[] = "maximized";
}  // namespace

void TobasWindowGeometry::LoadConfig(const tinyxml2::XMLElement*)
{
  if (title.empty()) {
    title = "Window Geometry";
  }

  const auto main_window = gz::gui::App()->findChild<gz::gui::MainWindow*>();
  if (!main_window || !main_window->QuickWindow()) {
    gzerr << "TobasWindowGeometry requires a main window.\n";
    return;
  }

  window_ = main_window->QuickWindow();
  settings_.beginGroup("window");
  save_timer_.setSingleShot(true);
  connect(&save_timer_, &QTimer::timeout, this, &TobasWindowGeometry::save);

  // Gazebo applies its GUI configuration after loading plugins. Restore after that has finished.
  QTimer::singleShot(0, this, &TobasWindowGeometry::initialize);
}

bool TobasWindowGeometry::eventFilter(QObject* _obj, QEvent* _event)
{
  if (_obj == window_) {
    if (_event->type() == QEvent::Move || _event->type() == QEvent::Resize || _event->type() == QEvent::WindowStateChange) {
      // Collect related geometry and state events before reading the final window state.
      save_timer_.start(0);
    }
    else if (_event->type() == QEvent::Close) {
      save();
    }
  }
  return gz::gui::Plugin::eventFilter(_obj, _event);
}

void TobasWindowGeometry::initialize()
{
  if (!window_) {
    return;
  }

  auto geometry = settings_.value(kGeometryKey).toRect();
  if (geometry.isValid()) {
    // Select the saved screen from global coordinates, falling back to the primary screen.
    auto screen = QGuiApplication::screenAt(geometry.center());
    if (!screen) {
      screen = QGuiApplication::screenAt(geometry.topLeft());
    }
    if (!screen) {
      screen = QGuiApplication::primaryScreen();
    }
    if (screen) {
      window_->setScreen(screen);
      // Leave space for the window manager's title bar and borders.
      const auto available = screen->availableGeometry().marginsRemoved(window_->frameMargins());
      if (available.isValid()) {
        geometry.setSize(geometry.size().boundedTo(available.size()));
        geometry.moveLeft(std::clamp(geometry.x(), available.left(), available.right() - geometry.width() + 1));
        geometry.moveTop(std::clamp(geometry.y(), available.top(), available.bottom() - geometry.height() + 1));
      }
    }
    if (QGuiApplication::platformName().startsWith("wayland")) {
      // Native Wayland does not expose global window positioning.
      window_->resize(geometry.size());
    }
    else {
      window_->setGeometry(geometry);
    }
  }

  normal_geometry_ = window_->geometry();
  if (const auto screen = window_->screen()) {
    normal_screen_position_ = screen->geometry().topLeft();
  }

  if (settings_.value(kMaximizedKey, false).toBool()) {
    window_->showMaximized();
  }

  window_->installEventFilter(this);
  connect(window_, &QWindow::screenChanged, this, [this](QScreen*) { save_timer_.start(0); });
  connect(QCoreApplication::instance(), &QCoreApplication::aboutToQuit, this, &TobasWindowGeometry::save);
}

void TobasWindowGeometry::save()
{
  if (!window_ || !window_->isVisible() || (window_->windowStates() & (Qt::WindowMinimized | Qt::WindowFullScreen))) {
    return;
  }

  const auto maximized = window_->windowStates().testFlag(Qt::WindowMaximized);
  const auto screen = window_->screen();
  if (!maximized) {
    normal_geometry_ = window_->geometry();
  }
  else if (screen) {
    // Moving a maximized window must also move the geometry used when it is restored to normal size.
    normal_geometry_.translate(screen->geometry().topLeft() - normal_screen_position_);
  }
  if (!normal_geometry_.isValid()) {
    return;
  }

  settings_.setValue(kGeometryKey, normal_geometry_);
  settings_.setValue(kMaximizedKey, maximized);
  if (screen) {
    normal_screen_position_ = screen->geometry().topLeft();
  }

  // Terminate may stop Gazebo without closing its window, so flush while it is running.
  settings_.sync();
  if (settings_.status() != QSettings::NoError) {
    qWarning() << "Failed to save Gazebo window geometry to" << settings_.fileName();
  }
}
}  // namespace tobas::gazebo

GZ_ADD_PLUGIN(tobas::gazebo::TobasWindowGeometry, gz::gui::Plugin)
