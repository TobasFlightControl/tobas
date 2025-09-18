#include "tobas_flight_log_gui/log_viewer/plots/joint_position_plot.hpp"

#include <ranges>

#include <tobas_qt_tools/util.hpp>
#include <tobas_ros2_tools/time.hpp>

namespace gui
{
namespace log
{
JointPositionPlotWidget::JointPositionPlotWidget()
{
  grid_ = new QGridLayout();
  setLayout(grid_);
}

void JointPositionPlotWidget::clear()
{
  // レイアウトとコンテナに同じウィジェットが含まれる場合は，コンテナ，レイアウトの順にクリアする必要がある．
  plots_.clear();
  cur_pos_curves_.clear();
  tar_pos_curves_.clear();
  qt::clearLayout(grid_);

  name2idx_.clear();
}

void JointPositionPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void JointPositionPlotWidget::setData(
  const QVector<tobas_msgs::msg::JointStateArray>& cur_msgs,
  const QVector<tobas_msgs::msg::JointCommandArray>& tar_msgs)
{
  updateCurrentSamples(cur_msgs);
  updateTargetSamples(tar_msgs);

  for (auto& plot : plots_) {
    plot->replot();
  }
}

size_t JointPositionPlotWidget::numJoints() const
{
  return name2idx_.size();
}

void JointPositionPlotWidget::addJoint(const std::string& name)
{
  const auto idx = numJoints();

  if (!name2idx_.insert({ name, idx }).second) {
    qWarning() << "Joint \"" << QString::fromStdString(name) << "\" already exists.";
    return;
  }

  const auto plot = new QwtPlot2();
  plot->setAxisNoLabel(QwtPlot::xBottom);
  grid_->addWidget(plot, idx / 2, idx % 2, 1, 1);  // N行2列の格子状に配置

  qwt::QwtPlotCurveWrapper cur_pos_curve("Current Position (" + QString::fromStdString(name) + ")");
  cur_pos_curve.setPen(kCurrentValueColor, kLineWidth);
  cur_pos_curve.attach(plot);

  qwt::QwtPlotCurveWrapper tar_pos_curve("Target Position (" + QString::fromStdString(name) + ")");
  tar_pos_curve.setPen(kTargetValueColor, kLineWidth);
  tar_pos_curve.attach(plot);

  plots_.append(plot);
  cur_pos_curves_.append(cur_pos_curve);
  tar_pos_curves_.append(tar_pos_curve);
}

void JointPositionPlotWidget::updateCurrentSamples(const QVector<tobas_msgs::msg::JointStateArray>& msgs)
{
  QVector<QVector<double>> t_data(numJoints());
  QVector<QVector<double>> pos_data(numJoints());

  for (const auto& msg : msgs) {
    for (const auto& elem : msg.states) {
      if (!name2idx_.contains(elem.name)) {
        addJoint(elem.name);
        t_data.append(QVector<double>{});
        pos_data.append(QVector<double>{});
      }

      const auto& idx = name2idx_.at(elem.name);

      t_data[idx].append(ros2::seconds(msg.header.stamp));
      pos_data[idx].append(elem.position);
    }
  }

  for (size_t i = 0; i < numJoints(); ++i) {
    cur_pos_curves_[i].setSamples(t_data[i], pos_data[i]);
  }
}

void JointPositionPlotWidget::updateTargetSamples(const QVector<tobas_msgs::msg::JointCommandArray>& msgs)
{
  QVector<QVector<double>> t_data(numJoints());
  QVector<QVector<double>> pos_data(numJoints());

  for (const auto& msg : msgs) {
    for (const auto& elem : msg.commands) {
      if (!name2idx_.contains(elem.name)) {
        addJoint(elem.name);
        t_data.append(QVector<double>{});
        pos_data.append(QVector<double>{});
      }

      const auto& idx = name2idx_[elem.name];

      t_data[idx].append(ros2::seconds(msg.header.stamp));
      pos_data[idx].append(elem.data);
    }
  }

  for (size_t i = 0; i < numJoints(); ++i) {
    tar_pos_curves_[i].setSamples(t_data[i], pos_data[i]);
  }
}
}  // namespace log
}  // namespace gui
