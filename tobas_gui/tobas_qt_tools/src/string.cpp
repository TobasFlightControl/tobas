#include "tobas_qt_tools/string.hpp"

namespace qt
{
QStringList stringListFromStdToQt(const std::vector<std::string>& list)
{
  QStringList res;
  for (const auto& item : list) {
    res.append(QString::fromStdString(item));
  }
  return res;
}
}  // namespace qt
