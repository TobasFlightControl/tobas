#pragma once

#include <QGridLayout>

#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>

#include "./common.hpp"

namespace gui
{
namespace log
{
class JointPositionPlotWidget : public BasePlotWidget
{
  Q_OBJECT

public:
  explicit JointPositionPlotWidget();

  void clear() override;
  void setTimeScale(double t_start, double t_stop) override;

  void setData(
    const QVector<tobas_msgs::msg::JointStateArray>& cur_msgs,
    const QVector<tobas_msgs::msg::JointCommandArray>& tar_msgs);

private:
  QVector<QwtPlot2*> plots_;

  QVector<qwt::QwtPlotCurveWrapper> cur_curves_;
  QVector<qwt::QwtPlotCurveWrapper> tar_curves_;

  QGridLayout* grid_;

  std::unordered_map<std::string, size_t> name2idx_;  // Joint Name -> Index

  size_t numJoints() const;

  void addJoint(const std::string& name);

  void updateCurrentSamples(const QVector<tobas_msgs::msg::JointStateArray>& msgs);
  void updateTargetSamples(const QVector<tobas_msgs::msg::JointCommandArray>& msgs);
};
}  // namespace log
}  // namespace gui
