#pragma once

#include <QGridLayout>

#include <tobas_msgs/msg/rotor_state_array.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class RotorSpeedPlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit RotorSpeedPlotWidget();

  void setTimeScale(double t_start, double t_stop);
  void setData(const QVector<tobas_msgs::msg::RotorStateArray>& msgs);

private:
  QVector<QwtPlot2*> plots_;

  QVector<qwt::QwtPlotCurveWrapper*> cur_speed_curves_;
  QVector<qwt::QwtPlotCurveWrapper*> tar_speed_curves_;

  QGridLayout* grid_;

  size_t num_rotors_;

  void clear();
};
}  // namespace log
}  // namespace gui
