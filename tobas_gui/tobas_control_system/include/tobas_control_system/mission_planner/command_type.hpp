// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QString>

#include <tobas_mission_items/mission.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
QString commandToText(mission::Type cmd);
mission::Type textToCommand(const QString& text);
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
