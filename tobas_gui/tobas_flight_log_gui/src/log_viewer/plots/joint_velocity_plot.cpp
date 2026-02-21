#include "tobas_flight_log_gui/log_viewer/plots/joint_velocity_plot.hpp"

#include <ranges>

#include <tobas_qt_tools/util.hpp>
#include <tobas_ros2_tools/time.hpp>

namespace gui
{
namespace log
{
JointVelocityPlotWidget::JointVelocityPlotWidget()
{
  grid_ = new qt::GridLayout();
  setLayout(grid_);
}

void JointVelocityPlotWidget::clear()
{
  // レイアウトとコンテナに同じウィジェットが含まれる場合は，コンテナ，レイアウトの順にクリアする必要がある．
  plots_.clear();
  cur_curves_.clear();
  tar_curves_.clear();
  grid_->clear();

  name2idx_.clear();
}

void JointVelocityPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void JointVelocityPlotWidget::setData(
  const QVector<tobas_msgs::msg::JointStateArray>& cur_msgs,
  const QVector<tobas_msgs::msg::JointCommandArray>& tar_msgs)
{
  updateCurrentSamples(cur_msgs);
  updateTargetSamples(tar_msgs);

  for (auto& plot : plots_) {
    plot->replot();
  }
}

size_t JointVelocityPlotWidget::numJoints() const
{
  return name2idx_.size();
}

void JointVelocityPlotWidget::addJoint(const std::string& name)
{
  const auto idx = numJoints();

  if (!name2idx_.insert({ name, idx }).second) {
    qWarning() << "Joint" << QString::fromStdString(name) << "already exists.";
    return;
  }

  const auto plot = new QwtPlot2();
  plot->setAxisNoLabel(QwtPlot::xBottom);

  // ウィジェットをN行2列の格子状に配置
  const auto row = idx / 2;
  const auto col = idx % 2;
  grid_->addWidget(plot, row, col, 1, 1);
  grid_->setRowStretch(row, 1);
  grid_->setColumnStretch(col, 1);

  qwt::QwtPlotCurveWrapper cur_curve("Current Velocity (" + QString::fromStdString(name) + ")");
  cur_curve.setPen(kCurrentValueColor, kLineWidth);
  cur_curve.attach(plot);

  qwt::QwtPlotCurveWrapper tar_curve("Target Velocity (" + QString::fromStdString(name) + ")");
  tar_curve.setPen(kTargetValueColor, kLineWidth);
  tar_curve.attach(plot);

  plots_.append(plot);
  cur_curves_.append(cur_curve);
  tar_curves_.append(tar_curve);
}

void JointVelocityPlotWidget::updateCurrentSamples(const QVector<tobas_msgs::msg::JointStateArray>& msgs)
{
  QVector<QVector<double>> times(numJoints());
  QVector<QVector<double>> values(numJoints());

  for (const auto& msg : msgs) {
    for (const auto& elem : msg.states) {
      if (!name2idx_.contains(elem.name)) {
        addJoint(elem.name);
        times.append(QVector<double>{});
        values.append(QVector<double>{});
      }

      const auto& idx = name2idx_.at(elem.name);

      times[idx].append(ros2::seconds(msg.header.stamp));
      values[idx].append(elem.velocity);
    }
  }

  for (size_t i = 0; i < numJoints(); ++i) {
    cur_curves_[i].setSamples(times[i], values[i]);
  }
}

void JointVelocityPlotWidget::updateTargetSamples(const QVector<tobas_msgs::msg::JointCommandArray>& msgs)
{
  QVector<QVector<double>> times(numJoints());
  QVector<QVector<double>> values(numJoints());

  for (const auto& msg : msgs) {
    for (const auto& elem : msg.commands) {
      if (!name2idx_.contains(elem.name)) {
        addJoint(elem.name);
        times.append(QVector<double>{});
        values.append(QVector<double>{});
      }

      const auto& idx = name2idx_[elem.name];

      times[idx].append(ros2::seconds(msg.header.stamp));
      values[idx].append(elem.data);
    }
  }

  for (size_t i = 0; i < numJoints(); ++i) {
    tar_curves_[i].setSamples(times[i], values[i]);
  }
}
}  // namespace log
}  // namespace gui
