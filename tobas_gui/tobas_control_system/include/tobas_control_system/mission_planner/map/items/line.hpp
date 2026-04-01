// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QtPositioning/QGeoCoordinate>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace map
{
class LineModel : public MapItemModel<double, double, double, double>
{
public:
  QString modelName() const override;
  QByteArrayList argNames() const override;
};
}  // namespace map
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
