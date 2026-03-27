#include "tobas_qt_tools/widgets/label.hpp"

#include "tobas_qt_tools/color.hpp"
#include "tobas_qt_tools/font.hpp"

namespace tobas
{
namespace qt
{
Label::Label(const QString& text, int point_size, int weight, bool italic, QWidget* parent) : super(text, parent)
{
  setFont(DefaultFont(point_size, weight, italic));
}

void Label::setTextColor(const QString color)
{
  setStyleSheet("color: " + color + ";");
}

void Label::setTextColor(const Qt::GlobalColor color)
{
  setTextColor(toCssColor(color));
}
}  // namespace qt
}  // namespace tobas
