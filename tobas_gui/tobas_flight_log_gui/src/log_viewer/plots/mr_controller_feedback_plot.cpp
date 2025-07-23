#include "tobas_flight_log_gui/log_viewer/plots/mr_controller_feedback_plot.hpp"

#include <QGridLayout>

#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_kdl/rotation.hpp>
#include <tobas_ros2_tools/time.hpp>

namespace gui
{
namespace log
{
MRControllerFeedbackPlotWidget::MRControllerFeedbackPlotWidget()
  : ei_curves_{ "X Integral Error [ms]",      "Y Integral Error [ms]",       "Z Integral Error [ms]",
                "Roll Integral Error [rads]", "Pitch Integral Error [rads]", "Yaw Integral Error [rads]" }
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  for (size_t i = 0; i < kNumAxes; ++i) {
    ei_plots_[i] = new QwtPlot2();
    ei_plots_[i]->setAxisNoLabel(QwtPlot::xBottom);
    grid->addWidget(ei_plots_[i], i % 3, i / 3, 1, 1);

    ei_curves_[i].setPen(kColorXYZ[i % 3], kLineWidth);
    ei_curves_[i].attach(ei_plots_[i]);
  }
}

void MRControllerFeedbackPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : ei_plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void MRControllerFeedbackPlotWidget::setData(const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, kNumAxes> ei_data;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));

    const auto& pos_ei = msg.position_integral_error;
    ei_data[0].push_back(pos_ei.x);
    ei_data[1].push_back(pos_ei.y);
    ei_data[2].push_back(pos_ei.z);

    const auto& rot_ei = msg.angle_integral_error;
    ei_data[3].push_back(rot_ei.x);
    ei_data[4].push_back(rot_ei.y);
    ei_data[5].push_back(rot_ei.z);
  }

  for (size_t i = 0; i < kNumAxes; ++i) {
    ei_curves_[i].setSamples(t_data, ei_data[i]);
    ei_plots_[i]->replot();
  }
}
}  // namespace log
}  // namespace gui
