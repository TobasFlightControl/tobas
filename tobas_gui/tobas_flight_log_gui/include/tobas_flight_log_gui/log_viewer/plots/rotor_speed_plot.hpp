#pragma once

#include <QGridLayout>

#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>

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
  void setData(
    const QVector<tobas_msgs::msg::RotorStateArray>& cur_msgs,
    const QVector<tobas_msgs::msg::RotorSpeedArray>& tar_msgs);

private:
  QVector<QwtPlot2*> plots_;

  QVector<qwt::QwtPlotCurveWrapper::SharedPtr> cur_speed_curves_;
  QVector<qwt::QwtPlotCurveWrapper::SharedPtr> tar_speed_curves_;

  QGridLayout* grid_;

  size_t num_rotors_;
  QMap<uint8_t, size_t> channel2idx_;

  void clear();
  void updateInternalDataStructures(const tobas_msgs::msg::RotorStateArray& msg);

  void updateCurrentSpeedSamples(const QVector<tobas_msgs::msg::RotorStateArray>& msgs);
  void updateTargetSpeedSamples(const QVector<tobas_msgs::msg::RotorSpeedArray>& msgs);
};
}  // namespace log
}  // namespace gui
