#pragma once

#include <QString>

namespace qt
{
QString getBaseName(const QString& path);

/* Get the absolute path to the resource directory of this package. */
QString getResourcePath();
}  // namespace qt
