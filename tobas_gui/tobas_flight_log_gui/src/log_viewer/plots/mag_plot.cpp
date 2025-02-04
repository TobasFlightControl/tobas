#include <QVBoxLayout>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_flight_log_gui/log_viewer/plots/mag_plot.hpp"

namespace gui
{
namespace log
{
MagPlotWidget::MagPlotWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  mag_curves_[0] = std::make_shared<qwt::QwtPlotCurveWrapper>("Mag X");
  mag_curves_[1] = std::make_shared<qwt::QwtPlotCurveWrapper>("Mag Y");
  mag_curves_[2] = std::make_shared<qwt::QwtPlotCurveWrapper>("Mag Z");

  for (size_t i = 0; i < 3; ++i)
  {
    mag_plots_[i] = new QwtPlot2();
    mag_curves_[i]->setPen(kColorXYZ[i], kLineWidth);
    mag_curves_[i]->attach(mag_plots_[i]);
    rows->addWidget(mag_plots_[i]);
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
