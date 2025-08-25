#pragma once

#include <QString>

namespace qt
{
QString getBaseName(const QString& path);

/* Get the absolute path to the resource directory. */
QString getResourcePath();
}  // namespace qt
