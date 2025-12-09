#pragma once

#include <tobas_qt_tools/layouts/grid_layout.hpp>

#include <tobas_msgs/msg/rotor_state_array.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class RotorLinkPlotWidget : public BasePlotWidget
{
  Q_OBJECT

public:
  explicit RotorLinkPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_msgs::msg::RotorStateArray>& msgs);

private:
  QVector<QwtPlot2*> plots_;
  QVector<qwt::QwtPlotCurveWrapper> curves_;

  qt::GridLayout* grid_;

  size_t num_rotors_;                                 // The number of rotors
  std::unordered_map<std::string, size_t> name2idx_;  // Link Name -> Index

  bool updateInternalDataStructures(const tobas_msgs::msg::RotorStateArray& msg);
};
}  // namespace log
}  // namespace gui
