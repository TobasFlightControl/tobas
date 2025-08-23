#pragma once

#include <QColor>
#include <QString>

namespace qt
{
QString toCssColor(Qt::GlobalColor c, bool with_alpha = false);
}  // namespace qt
