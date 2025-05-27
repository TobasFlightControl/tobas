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
  : pos_ei_curves_{ "X Integral Error", "Y Integral Error", "Z Integral Error" }
  , rot_ei_curves_{ "Roll Integral Error", "Pitch Integral Error", "Yaw Integral Error" }
{
  const auto grid = new QGridLayout();
  setLayout(grid);

  for (size_t i = 0; i < 3; ++i) {
    pos_ei_plots_[i] = new QwtPlot2();
    pos_ei_curves_[i].setPen(kColorXYZ[i], kLineWidth);
    pos_ei_curves_[i].attach(pos_ei_plots_[i]);
    grid->addWidget(pos_ei_plots_[i], i, 0);

    rot_ei_plots_[i] = new QwtPlot2();
    rot_ei_curves_[i].setPen(kColorXYZ[i], kLineWidth);
    rot_ei_curves_[i].attach(rot_ei_plots_[i]);
    grid->addWidget(rot_ei_plots_[i], i, 1);
  }
}

void MRControllerFeedbackPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (size_t i = 0; i < 3; ++i) {
    pos_ei_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
    rot_ei_plots_[i]->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void MRControllerFeedbackPlotWidget::setData(const QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback>& msgs)
{
  QVector<double> t_data;
  std::array<QVector<double>, 3> pos_ei_data;
  std::array<QVector<double>, 3> rot_ei_data;

  for (const auto& msg : msgs) {
    t_data.push_back(ros2::seconds(msg.header.stamp));

    const auto& pos_ei = msg.position_integral_error;
    pos_ei_data[0].push_back(pos_ei.x);
    pos_ei_data[1].push_back(pos_ei.y);
    pos_ei_data[2].push_back(pos_ei.z);

    const auto& rot_ei = msg.angle_integral_error;
    rot_ei_data[0].push_back(rot_ei.x);
    rot_ei_data[1].push_back(rot_ei.y);
    rot_ei_data[2].push_back(rot_ei.z);
  }

  for (size_t i = 0; i < 3; ++i) {
    pos_ei_curves_[i].setSamples(t_data, pos_ei_data[i]);
    pos_ei_plots_[i]->replot();

    rot_ei_curves_[i].setSamples(t_data, rot_ei_data[i]);
    rot_ei_plots_[i]->replot();
  }
}
}  // namespace log
}  // namespace gui
