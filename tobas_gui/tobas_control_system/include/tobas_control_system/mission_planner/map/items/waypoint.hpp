// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QtPositioning/QGeoCoordinate>

#include "./base.hpp"

namespace tobas::gui::ctrl::map
{
class WaypointModel : public MapItemModel<int, QGeoCoordinate, double, QString>
{
public:
  QString modelName() const override;
  QByteArrayList argNames() const override;
};
}  // namespace tobas::gui::ctrl::map
