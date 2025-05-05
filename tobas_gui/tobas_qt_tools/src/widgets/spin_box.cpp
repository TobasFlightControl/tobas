#include "tobas_qt_tools/widgets/spin_box.hpp"

#include <QTimer>
#include <QWheelEvent>

namespace qt
{
SpinBox::SpinBox(QWidget* parent) : super(parent)
{
  setMinimum(std::numeric_limits<int>::lowest());
  setMaximum(std::numeric_limits<int>::max());
}

void SpinBox::wheelEvent(QWheelEvent* event)
{
  event->ignore();
}

void SpinBox::focusInEvent(QFocusEvent* event)
{
  super::focusInEvent(event);
  QTimer::singleShot(0, this, &DoubleSpinBox::selectAll);
}

DoubleSpinBox::DoubleSpinBox(QWidget* parent) : super(parent)
{
  setMinimum(std::numeric_limits<double>::lowest());
  setMaximum(std::numeric_limits<double>::max());
}

void DoubleSpinBox::wheelEvent(QWheelEvent* event)
{
  event->ignore();
}

void DoubleSpinBox::focusInEvent(QFocusEvent* event)
{
  super::focusInEvent(event);
  QTimer::singleShot(0, this, &DoubleSpinBox::selectAll);
}
}  // namespace qt
