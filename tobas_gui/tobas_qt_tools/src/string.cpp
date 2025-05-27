#include "tobas_qt_tools/string.hpp"

namespace qt
{
QStringList stringListFromStdToQt(const std::vector<std::string>& src)
{
  QStringList res;
  for (const auto& item : src) {
    res.append(QString::fromStdString(item));
  }
  return res;
}

std::vector<std::string> stringListFromQtToStd(const QStringList& src)
{
  std::vector<std::string> res;
  for (const auto& item : src) {
    res.push_back(item.toStdString());
  }
  return res;
}
}  // namespace qt
