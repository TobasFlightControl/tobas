#include <QGridLayout>
#include <qwt/qwt_plot_curve.h>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/mag_plot.hpp"
#include "tobas_flight_log_gui/log_viewer/plots/util.hpp"
#include "tobas_flight_log_gui/log_viewer/constants.hpp"

namespace gui
{
namespace log
{
MagPlotWidget::MagPlotWidget()
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  mag_curves_[0] = new QwtPlotCurve("Mag X");
  mag_curves_[1] = new QwtPlotCurve("Mag Y");
  mag_curves_[2] = new QwtPlotCurve("Mag Z");

  for (size_t i = 0; i < 3; ++i)
  {
    mag_plots_[i] = new qt::QwtPlot2();
    mag_curves_[i]->setPen(kColor, kLineWidth);
    mag_curves_[i]->attach(mag_plots_[i]);
    enableLegend(mag_curves_[i]);
    grid->addWidget(mag_plots_[i], i, 0);
  }
}

void MagPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (size_t i = 0; i < 3; ++i)
    mag_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
}

void MagPlotWidget::setData(const QVector<tobas_msgs::msg::MagneticFieldWithCovarianceStamped>& mag_msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, 3> mag_data;

  for (const auto& mag : mag_msgs)
  {
    t_data.push_back(ros2::seconds(mag.header.stamp));

    mag_data[0].push_back(mag.mag.mag.x);
    mag_data[1].push_back(mag.mag.mag.y);
    mag_data[2].push_back(mag.mag.mag.z);
  }

  for (size_t i = 0; i < 3; ++i)
  {
    mag_curves_[i]->setSamples(t_data, mag_data[i]);
    mag_plots_[i]->replot();
  }
}
}  // namespace log
}  // namespace gui
