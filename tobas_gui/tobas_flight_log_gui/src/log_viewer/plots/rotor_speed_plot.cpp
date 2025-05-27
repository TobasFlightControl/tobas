#include "tobas_flight_log_gui/log_viewer/plots/rotor_speed_plot.hpp"

#include <ranges>

#include <tobas_qt_tools/util.hpp>
#include <tobas_ros2_tools/time.hpp>

namespace gui
{
namespace log
{
RotorSpeedPlotWidget::RotorSpeedPlotWidget()
{
  grid_ = new QGridLayout();
  setLayout(grid_);
}

void RotorSpeedPlotWidget::clear()
{
  // XXX: レイアウトとコンテナに同じウィジェットが含まれる場合は，コンテナ，レイアウトの順にクリアする必要がある．
  plots_.clear();
  cur_speed_curves_.clear();
  tar_speed_curves_.clear();
  qt::clearLayout(grid_);

  num_rotors_ = 0;
  name2idx_.clear();
}

void RotorSpeedPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void RotorSpeedPlotWidget::setData(
  const QVector<tobas_msgs::msg::RotorStateArray>& cur_msgs,
  const QVector<tobas_msgs::msg::RotorSpeedArray>& tar_msgs)
{
  if (cur_msgs.size() == 0) {
    return;
  }

  const auto& first_msg = cur_msgs.at(0);
  if (first_msg.states.size() != num_rotors_) {
    if (!updateInternalDataStructures(first_msg)) {
      return;
    }
  }

  updateCurrentSpeedSamples(cur_msgs);
  updateTargetSpeedSamples(tar_msgs);

  for (auto& plot : plots_) {
    plot->replot();
  }
}

bool RotorSpeedPlotWidget::updateInternalDataStructures(const tobas_msgs::msg::RotorStateArray& msg)
{
  clear();

  for (const auto& [idx, elem] : std::views::enumerate(msg.states)) {
    if (elem.link_name.empty()) {
      qWarning() << "Rotor link name is empty.";
      return false;
    }

    if (!name2idx_.insert({ elem.link_name, idx }).second) {
      qWarning() << "Rotor \"" << QString::fromStdString(elem.link_name) << "\" is duplicated.";
      return false;
    }

    ++num_rotors_;

    plots_.push_back(new QwtPlot2());

    // N行2列の格子状に配置
    grid_->addWidget(plots_.back(), idx / 2, idx % 2);

    cur_speed_curves_.push_back("Current Speed (" + QString::fromStdString(elem.link_name) + ")");
    cur_speed_curves_.back().setPen(kCurrentValueColor, kLineWidth);
    cur_speed_curves_.back().attach(plots_.back());

    tar_speed_curves_.push_back("Target Speed (" + QString::fromStdString(elem.link_name) + ")");
    tar_speed_curves_.back().setPen(kTargetValueColor, kLineWidth);
    tar_speed_curves_.back().attach(plots_.back());
  }

  return true;
}

void RotorSpeedPlotWidget::updateCurrentSpeedSamples(const QVector<tobas_msgs::msg::RotorStateArray>& msgs)
{
  QVector<QVector<double>> t_data(num_rotors_);
  QVector<QVector<double>> speed_data(num_rotors_);

  for (const auto& msg : msgs) {
    if (msg.states.size() != num_rotors_) {
      qWarning() << "The number of rotors mismatch.";
      continue;
    }

    for (const auto& elem : msg.states) {
      if (!name2idx_.contains(elem.link_name)) {
        qWarning() << "Rotor \"" << QString::fromStdString(elem.link_name) << "\" is not registered.";
        continue;
      }

      if (elem.status == tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE) {
        continue;
      }

      const auto& idx = name2idx_[elem.link_name];

      t_data[idx].push_back(ros2::seconds(msg.header.stamp));
      speed_data[idx].push_back(elem.speed);
    }
  }

  for (size_t i = 0; i < num_rotors_; ++i) {
    cur_speed_curves_[i].setSamples(t_data[i], speed_data[i]);
  }
}

void RotorSpeedPlotWidget::updateTargetSpeedSamples(const QVector<tobas_msgs::msg::RotorSpeedArray>& msgs)
{
  QVector<QVector<double>> t_data(num_rotors_);
  QVector<QVector<double>> speed_data(num_rotors_);

  for (const auto& msg : msgs) {
    if (msg.speeds.size() != num_rotors_) {
      qWarning() << "The number of rotors mismatch.";
      continue;
    }

    for (const auto& speed : msg.speeds) {
      if (!name2idx_.contains(speed.link_name)) {
        qWarning() << "Rotor \"" << QString::fromStdString(speed.link_name) << "\" is not registered.";
        continue;
      }

      const auto& idx = name2idx_[speed.link_name];

      t_data[idx].push_back(ros2::seconds(msg.header.stamp));
      speed_data[idx].push_back(speed.speed);
    }
  }

  for (size_t i = 0; i < num_rotors_; ++i) {
    tar_speed_curves_[i].setSamples(t_data[i], speed_data[i]);
  }
}
}  // namespace log
}  // namespace gui
