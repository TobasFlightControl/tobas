#pragma once

#include <QGridLayout>

#include <tobas_msgs/msg/ice_propulsion_system_command.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class PropellerPitchPlotWidget : public BasePlotWidget
{
  Q_OBJECT

public:
  explicit PropellerPitchPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(const QVector<tobas_msgs::msg::IcePropulsionSystemCommand>& msgs);

private:
  QVector<QwtPlot2*> plots_;
  QVector<qwt::QwtPlotCurveWrapper> curves_;
  QGridLayout* grid_;

  size_t num_rotors_;                                 // The number of rotors
  std::unordered_map<std::string, size_t> name2idx_;  // Link Name -> Index

  bool updateInternalDataStructures(const tobas_msgs::msg::IcePropulsionSystemCommand& msg);
};
}  // namespace log
}  // namespace gui
