#include "tobas_qt_tools/widgets/slider.hpp"

#include <QWheelEvent>

namespace tobas
{
namespace qt
{
void Slider::wheelEvent(QWheelEvent* event)
{
  event->ignore();
}
}  // namespace qt
}  // namespace tobas
