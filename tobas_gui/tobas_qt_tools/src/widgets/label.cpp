#include "tobas_qt_tools/widgets/label.hpp"

#include "tobas_qt_tools/font.hpp"

namespace qt
{
Label::Label(const QString& text, int point_size, int weight, bool italic, QWidget* parent) : super(text, parent)
{
  setFont(DefaultFont(point_size, weight, italic));
}
}  // namespace qt
