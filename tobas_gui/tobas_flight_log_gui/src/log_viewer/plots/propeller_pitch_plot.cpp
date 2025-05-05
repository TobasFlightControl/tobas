#include "tobas_flight_log_gui/log_viewer/plots/propeller_pitch_plot.hpp"

#include <ranges>

#include <tobas_ros2_tools/time.hpp>
#include <tobas_qt_tools/util.hpp>

namespace gui
{
namespace log
{
PropellerPitchPlotWidget::PropellerPitchPlotWidget()
{
  grid_ = new QGridLayout();
  setLayout(grid_);
}

void PropellerPitchPlotWidget::setTimeScale(double t_start, double t_stop)
{
  for (auto& plot : plots_) {
    plot->setAxisScale(QwtPlot::xBottom, t_start, t_stop);
  }
}

void PropellerPitchPlotWidget::setData(const QVector<tobas_msgs::msg::IcePropulsionSystemCommand>& msgs)
{
  if (msgs.size() == 0) {
    return;
  }

  const auto& first_msg = msgs.at(0);
  if (first_msg.pitch_angles.size() != num_rotors_) {
    updateInternalDataStructures(first_msg);
  }

  QVector<QVector<double>> t_data(num_rotors_);
  QVector<QVector<double>> pitch_data(num_rotors_);

  for (const auto& msg : msgs) {
    if (msg.pitch_angles.size() != num_rotors_) {
      qWarning() << "The number of VPPs mismatch.";
      continue;
    }

    for (const auto& elem : msg.pitch_angles) {
      if (!name2idx_.contains(elem.link_name)) {
        qWarning() << "VPP \"" << QString::fromStdString(elem.link_name) << "\" is not registered.";
        continue;
      }

      const auto& idx = name2idx_.at(elem.link_name);

      t_data[idx].push_back(ros2::seconds(msg.header.stamp));
      pitch_data[idx].push_back(elem.angle);
    }
  }

  for (size_t i = 0; i < num_rotors_; ++i) {
    curves_[i].setSamples(t_data[i], pitch_data[i]);
  }

  for (auto& plot : plots_) {
    plot->replot();
  }
}

void PropellerPitchPlotWidget::clear()
{
  // XXX: レイアウトとコンテナに同じウィジェットが含まれる場合は，コンテナ，レイアウトの順にクリアする必要がある．
  plots_.clear();
  curves_.clear();
  qt::clearLayout(grid_);

  num_rotors_ = 0;
  name2idx_.clear();
}

void PropellerPitchPlotWidget::updateInternalDataStructures(const tobas_msgs::msg::IcePropulsionSystemCommand& msg)
{
  clear();

  num_rotors_ = msg.pitch_angles.size();

  for (const auto& [idx, elem] : std::views::enumerate(msg.pitch_angles)) {
    if (!name2idx_.insert({ elem.link_name, idx }).second) {
      qWarning() << "VPP \"" << QString::fromStdString(elem.link_name) << "\" is duplicated.";
      continue;
    }

    plots_.push_back(new QwtPlot2());

    // N行2列の格子状に配置
    grid_->addWidget(plots_.back(), idx / 2, idx % 2);

    curves_.push_back("Pitch Angle (" + QString::fromStdString(elem.link_name) + ")");
    curves_.back().setPen(kCurrentValueColor, kLineWidth);
    curves_.back().attach(plots_.back());
  }
}
}  // namespace log
}  // namespace gui
