// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/map/items/line.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace map
{
QString LineModel::modelName() const
{
  return "LineModel";
}

QByteArrayList LineModel::argNames() const
{
  return { "latitude_1", "longitude_1", "latitude_2", "longitude_2" };
}
}  // namespace map
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
