#include "tobas_qt_tools/stream.hpp"

std::ostream& operator<<(std::ostream& os, const QString& arg)
{
  os << arg.toStdString();
  return os;
}
