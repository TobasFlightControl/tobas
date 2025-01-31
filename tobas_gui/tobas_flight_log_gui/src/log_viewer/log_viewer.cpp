#include <QVBoxLayout>

#include <tobas_string_tools/core.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_flight_log_gui/log_viewer/log_viewer.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace log
{
FlightLogViewerWidget::FlightLogViewerWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  for (size_t i = 0; i < plot_tabs_.size(); ++i)
  {
    plot_tabs_[i] = new PlotTabWidget();
    rows->addWidget(plot_tabs_[i]);
  }

  playback_ctrl_ = new PlaybackControlWidget();
  rows->addWidget(playback_ctrl_);

  connect(playback_ctrl_, &PlaybackControlWidget::timeChanged, this, &self::onPlaybackTimeChanged);

  reset();
}

void FlightLogViewerWidget::reset()
{
  log_path_.clear();
  reader_.close();

  for (auto& plot_tab : plot_tabs_)
    plot_tab->setTimeScale(0., kWindowDuration);

  playback_ctrl_->reset();
}

void FlightLogViewerWidget::setLogName(const QString& log_name)
{
  reset();

  log_path_ = ros2::expandUser(tobas::kROSBagDirHome) / log_name.toStdString();

  try
  {
    reader_.open(log_path_);
  }
  catch (const std::exception& e)
  {
    qt::qErrorBox(this, "Failed to open " + QString::fromStdString(log_path_) + ".");
    return;
  }

  const auto& metadata = reader_.get_metadata();
  const auto duration = metadata.duration.count() * 1e-9;  // [s]
  playback_ctrl_->setDuration(std::max(duration - kWindowDuration, 0.));

  setPlotData(0.);
}

void FlightLogViewerWidget::setPlotData(double time_from_start)
{
  if (log_path_.empty())
  {
    qWarning() << "Log path is not set.";
    return;
  }

  if (!fs::exists(log_path_))
  {
    qWarning() << "Log path " << QString::fromStdString(log_path_) << " does not exist.";
    return;
  }

  const auto& metadata = reader_.get_metadata();
  const auto record_start_time = metadata.starting_time.time_since_epoch().count();              // [ns]
  const auto window_start_time = record_start_time + static_cast<long>(time_from_start * 1e+9);  // [ns]
  const auto window_stop_time = window_start_time + static_cast<long>(kWindowDuration * 1e+9);   // [ns]

  // 初期時刻に移動
  reader_.seek(window_start_time);

  // データを仕分ける
  QVector<tobas_msgs::msg::Odometry> odom_data;
  QVector<tobas_msgs::msg::ImuWithCovarianceStamped> imu_data;
  QVector<tobas_msgs::msg::MagneticFieldWithCovarianceStamped> mag_data;
  // TODO
  while (reader_.has_next())
  {
    const auto msg = reader_.read_next();

    const auto& cur_time = msg->recv_timestamp;  // [ns]
    if (cur_time > window_stop_time)
      break;

    rclcpp::SerializedMessage ser_msg(*msg->serialized_data);

    try
    {
      if (str::endsWith(msg->topic_name, path::join("/", tobas::kOdometryTopic)))
      {
        odom_ser_.deserialize_message(&ser_msg, &odom_);
        odom_data.push_back(odom_);
      }
      else if (str::endsWith(msg->topic_name, path::join("/", tobas::kImuTopic)))
      {
        imu_ser_.deserialize_message(&ser_msg, &imu_);
        imu_data.push_back(imu_);
      }
      else if (str::endsWith(msg->topic_name, path::join("/", tobas::kMagTopic)))
      {
        mag_ser_.deserialize_message(&ser_msg, &mag_);
        mag_data.push_back(mag_);
      }
      // TODO
    }
    catch (const std::exception& e)
    {
      qWarning() << "Failed to deserialize " << QString::fromStdString(msg->topic_name) << ": " + QString(e.what());
    }
  }

  // データをプロット
  for (auto& plot_tab : plot_tabs_)
  {
    plot_tab->setTimeScale(window_start_time * 1e-9, window_stop_time * 1e-9);

    plot_tab->setPoseData(odom_data);
    plot_tab->setImuData(imu_data);
    plot_tab->setMagData(mag_data);
    // TODO
  }
}

void FlightLogViewerWidget::onPlaybackTimeChanged(double time_from_start)
{
  setPlotData(time_from_start);
}
}  // namespace log
}  // namespace gui
