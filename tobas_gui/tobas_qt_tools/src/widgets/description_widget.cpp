#include "tobas_qt_tools/widgets/description_widget.hpp"

#include "tobas_qt_tools/font.hpp"

namespace qt
{
DescriptionWidget::DescriptionWidget(const QString& text, int point_size, QWidget* parent) : super(text, parent)
{
  setFont(DefaultFont(point_size));
  setAlignment(Qt::AlignTop);
  setWordWrap(true);
  setOpenExternalLinks(true);
}
}  // namespace qt
