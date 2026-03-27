#pragma once

#include <QColor>
#include <QString>

namespace tobas
{
namespace qt
{
QString toCssColor(Qt::GlobalColor c, bool with_alpha = false);
}  // namespace qt
}  // namespace tobas
