// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <qnamespace.h>

namespace tobas
{
namespace gui
{
namespace log
{
static constexpr int kMaxVerticalAxisLabels = 6;
static constexpr int kMaxVerticalScaleSteps = kMaxVerticalAxisLabels - 1;

static constexpr double kLineWidth = 0.8;

static constexpr Qt::GlobalColor kCurrentValueColor = Qt::blue;
static constexpr Qt::GlobalColor kTargetValueColor = Qt::red;
static constexpr Qt::GlobalColor kRawValueColor = Qt::lightGray;
static constexpr Qt::GlobalColor kFilteredValueColor = Qt::cyan;
static constexpr Qt::GlobalColor kColorXYZ[] = { Qt::red, Qt::green, Qt::blue };
}  // namespace log
}  // namespace gui
}  // namespace tobas
