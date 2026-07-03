// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QObject>

#include <tobas_drone_core/propulsion_system/type.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
/**
 * @brief Common signals.
 * Use a separate interface class to loosely couple widgets across hierarchy levels.
 * This follows the same idea as ROS messages.
 */
class Signals : public QObject
{
  Q_OBJECT

Q_SIGNALS:
  void propulsionTypeChanged(const PropulsionSystem& type);
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
