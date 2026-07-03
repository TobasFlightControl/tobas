// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QFont>

namespace tobas
{
namespace qt
{
/* `QFont` that uses the default style by default. */
class DefaultFont : public QFont
{
public:
  explicit DefaultFont(int point_size = -1, int weight = -1, bool italic = false);
};
}  // namespace qt
}  // namespace tobas
