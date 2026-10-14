// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QString>

#include <tobas_mission_items/mission.hpp>

namespace tobas::gui::ctrl
{
QString commandToText(mission::Type _cmd);
mission::Type textToCommand(const QString& _text);
}  // namespace tobas::gui::ctrl
