#include "tobas_qt_tools/font.hpp"

namespace tobas
{
namespace qt
{
DefaultFont::DefaultFont(int point_size, int weight, bool italic)
{
  if (point_size > 0) {
    setPointSize(point_size);
  }

  if (weight >= 0) {
    setWeight(weight);
  }

  setItalic(italic);
}
}  // namespace qt
}  // namespace tobas
