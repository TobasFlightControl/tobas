#include "tobas_qt_tools/widgets/qwt_plot.hpp"

namespace qt
{
QwtPlot2::QwtPlot2(QWidget* parent) : super(parent)
{
  setMinimumSize(1, 1);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
}
}  // namespace qt
