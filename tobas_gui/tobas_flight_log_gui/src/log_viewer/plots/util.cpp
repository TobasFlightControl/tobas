#include <qwt/qwt_plot_curve.h>

#include "tobas_flight_log_gui/log_viewer/plots/util.hpp"

namespace gui
{
namespace log
{
void enableLegend(QwtPlotCurve* curve)
{
  curve->setLegendAttribute(QwtPlotCurve::LegendShowLine, true);
  curve->setLegendAttribute(QwtPlotCurve::LegendShowSymbol, true);
  curve->setItemAttribute(QwtPlotItem::Legend, true);
}
}  // namespace log
}  // namespace gui
