#include "tobas_flight_log_gui/log_viewer/plots/rotor_link_plot.hpp"

#include <ranges>

#include <tobas_qt_tools/util.hpp>
#include <tobas_ros2_tools/time.hpp>

namespace gui
{
namespace log
{
RotorLinkPlotWidget::RotorLinkPlotWidget()
{
  grid_ = new qt::GridLayout();
  setLayout(grid_);
}

void RotorLinkPlotWidget::clear()
{
  // レイアウトとコンテナに同じウィジェットが含まれる場合は，コンテナ，レイアウトの順にクリアする必要がある．
  plots_.clear();
  curves_.clear();
  grid_->clear();

  num_rotors_ = 0;
  name2idx_.clear();
}

void RotorLinkPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void RotorLinkPlotWidget::setData(const QVector<tobas_msgs::msg::RotorStateArray>& msgs)
{
  if (msgs.empty()) {
    return;
  }

  const auto& first_msg = msgs.first();
  if (first_msg.states.size() != num_rotors_) {
    if (!updateInternalDataStructures(first_msg)) {
      return;
    }
  }

  QVector<QVector<double>> t_data(num_rotors_);
  QVector<QVector<double>> value_data(num_rotors_);

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

      const auto& idx = name2idx_.at(elem.link_name);

      t_data[idx].push_back(ros2::seconds(msg.header.stamp));

      if (elem.status == tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE) {
        value_data[idx].push_back(1);
      }
      else {
        value_data[idx].push_back(0);
      }
    }
  }

  for (size_t i = 0; i < num_rotors_; ++i) {
    curves_[i].setSamples(t_data[i], value_data[i]);
  }

  for (auto& plot : plots_) {
    plot->replot();
  }
}

bool RotorLinkPlotWidget::updateInternalDataStructures(const tobas_msgs::msg::RotorStateArray& msg)
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

    const auto plot = new QwtPlot2();
    plot->setupBinaryPlot("OK", "ERR");

    // ウィジェットをN行2列の格子状に配置
    const auto row = idx / 2;
    const auto col = idx % 2;
    grid_->addWidget(plot, row, col, 1, 1);
    grid_->setRowStretch(row, 1);
    grid_->setColumnStretch(col, 1);

    qwt::QwtPlotCurveWrapper curve("Communication State (" + QString::fromStdString(elem.link_name) + ")");
    curve.setStyleSteps();
    curve.setPen(Qt::black, kLineWidth);
    curve.attach(plot);

    plots_.push_back(plot);
    curves_.push_back(curve);
  }

  return true;
}
}  // namespace log
}  // namespace gui
