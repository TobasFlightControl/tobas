#pragma once

#include <string>
#include <vector>

#include <QStringList>

namespace qt
{
QStringList stringListFromStdToQt(const std::vector<std::string>& src);
std::vector<std::string> stringListFromQtToStd(const QStringList& src);
}  // namespace qt
