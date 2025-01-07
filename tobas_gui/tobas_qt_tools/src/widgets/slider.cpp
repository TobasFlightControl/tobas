#include <QWheelEvent>

#include "tobas_qt_tools/widgets/slider.hpp"

namespace qt
{
void Slider::wheelEvent(QWheelEvent* event)
{
  event->ignore();
}
}  // namespace qt
