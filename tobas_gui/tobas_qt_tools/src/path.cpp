#include "tobas_qt_tools/path.hpp"

namespace qt
{
QString getBaseName(const QString& path)
{
  return path.left(path.lastIndexOf('.'));
}
}  // namespace qt
