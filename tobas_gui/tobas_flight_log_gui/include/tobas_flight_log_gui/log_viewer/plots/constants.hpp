#pragma once

#include <qnamespace.h>

namespace gui
{
namespace log
{
static constexpr double kLineWidth = 1.;

static constexpr Qt::GlobalColor kCurrentValueColor = Qt::blue;
static constexpr Qt::GlobalColor kTargetValueColor = Qt::red;
static constexpr Qt::GlobalColor kRawValueColor = Qt::lightGray;
static constexpr Qt::GlobalColor kFilteredValueColor = Qt::cyan;
static constexpr Qt::GlobalColor kColorXYZ[] = { Qt::red, Qt::green, Qt::blue };
}  // namespace log
}  // namespace gui
