#include "tobas_qt_tools/font.hpp"

namespace qt
{
DefaultFont::DefaultFont(int point_size, int weight, bool italic)
{
  setPointSize(point_size);
  setWeight(weight);
  setItalic(italic);
}
}  // namespace qt
