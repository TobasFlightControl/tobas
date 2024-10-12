#pragma once

#include <QStringList>

namespace qt
{
QStringList stringListFromStdToQt(const std::vector<std::string>& list);
}
