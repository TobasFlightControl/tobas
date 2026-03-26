#include "tobas_qt_tools/color.hpp"

namespace tobas
{
namespace qt
{
QString toCssColor(Qt::GlobalColor c, bool with_alpha)
{
  if (c == Qt::transparent) {
    return "transparent";  // 見やすさ重視
  }

  QColor q(c);

  if (with_alpha) {
    return q.name(QColor::HexArgb);  // #AARRGGBB
  }
  else {
    return q.name(QColor::HexRgb);  // #RRGGBB
  }
}
}  // namespace qt
}  // namespace tobas
