#include <QVBoxLayout>

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

  // rosbagの絶対パスを更新
  log_path_ = ros2::expandUser(tobas::kRosbagDirHome) / log_name.toStdString();

  // rosbagを開く
  try
  {
    reader_.open(log_path_);
  }
  catch (const std::exception& e)
  {
    qt::qErrorBox(this, "Failed to open " + QString::fromStdString(log_path_) + ".");
    return;
  }

  // ログの長さを更新
  const auto& metadata = reader_.get_metadata();
  const auto duration = metadata.duration.count() * 1e-9;  // [s]
  playback_ctrl_->setDuration(std::max(duration - kWindowDuration, 0.));

  // 時刻0のログを表示
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
  QVector<tobas_msgs::msg::Gnss> gnss_data;
  QVector<tobas_msgs::msg::Battery> battery_data;
  QVector<tobas_msgs::msg::RotorStateArray> cur_rotor_states_data;
  QVector<tobas_msgs::msg::RotorSpeedArray> tar_rotor_speeds_data;
  QVector<tobas_msgs::msg::Latency> sampling_time_data;
  QVector<tobas_msgs::msg::Latency> ctrl_latency_data;
  QVector<tobas_kdl_msgs::msg::WrenchStamped> dist_force_data;
  QVector<tobas_debug_msgs::msg::ObserverFeedback> obsv_fb_data;
  QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback> mr_ctrl_fb_data;
  while (reader_.has_next())
  {
    const auto msg = reader_.read_next();

    const auto& cur_time = msg->recv_timestamp;  // [ns]
    if (cur_time > window_stop_time)
      break;

    rclcpp::SerializedMessage ser_msg(*msg->serialized_data);

    try
    {
      if (msg->topic_name.ends_with(path::join("/", tobas::kOdometryTopic)))
      {
        odom_ser_.deserialize_message(&ser_msg, &odom_);
        odom_data.push_back(odom_);
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kImuTopic)))
      {
        imu_ser_.deserialize_message(&ser_msg, &imu_);
        imu_data.push_back(imu_);
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kMagTopic)))
      {
        mag_ser_.deserialize_message(&ser_msg, &mag_);
        mag_data.push_back(mag_);
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kGnssTopic)))
      {
        gnss_ser_.deserialize_message(&ser_msg, &gnss_);
        gnss_data.push_back(gnss_);
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kBatteryTopic)))
      {
        battery_ser_.deserialize_message(&ser_msg, &battery_);
        battery_data.push_back(battery_);
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kRotorStatesTopic)))
      {
        cur_rotor_states_ser_.deserialize_message(&ser_msg, &rotor_states_);
        cur_rotor_states_data.push_back(rotor_states_);
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kRotorSpeedsCmdTopic)))
      {
        tar_rotor_speeds_ser_.deserialize_message(&ser_msg, &rotor_speeds_);
        tar_rotor_speeds_data.push_back(rotor_speeds_);
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kImuSamplingTimeTopic)))
      {
        latency_ser_.deserialize_message(&ser_msg, &latency_);
        sampling_time_data.push_back(latency_);
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kControlLatencyTopic)))
      {
        latency_ser_.deserialize_message(&ser_msg, &latency_);
        ctrl_latency_data.push_back(latency_);
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kDisturbanceForceTopic)))
      {
        wrench_ser_.deserialize_message(&ser_msg, &wrench_);
        dist_force_data.push_back(wrench_);
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kObsvFeedbackTopic)))
      {
        obsv_fb_ser_.deserialize_message(&ser_msg, &obsv_fb_);
        obsv_fb_data.push_back(obsv_fb_);
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kMRCtrlFeedbackTopic)))
      {
        mr_ctrl_fb_ser_.deserialize_message(&ser_msg, &mr_ctrl_fb_);
        mr_ctrl_fb_data.push_back(mr_ctrl_fb_);
      }
    }
    catch (const std::exception& e)
    {
      qWarning() << "Failed to deserialize " << QString::fromStdString(msg->topic_name) << ": " + QString(e.what());
    }
  }

  // データをプロット
  for (auto& plot_tab : plot_tabs_)
  {
    // XXX: データの設定の前に範囲を指定しないと若干プロットが崩れる
    plot_tab->setTimeScale(window_start_time * 1e-9, window_stop_time * 1e-9);

    plot_tab->setPoseData(odom_data, mr_ctrl_fb_data);
    plot_tab->setTwistData(odom_data, mr_ctrl_fb_data);
    plot_tab->setImuData(imu_data);
    plot_tab->setMagData(mag_data);
    plot_tab->setGnssData(gnss_data);
    plot_tab->setBatteryData(battery_data);
    plot_tab->setRotorSpeedData(cur_rotor_states_data, tar_rotor_speeds_data);
    plot_tab->setSamplingTimeData(sampling_time_data);
    plot_tab->setControlLatencyData(ctrl_latency_data);
    plot_tab->setDisturbanceForceData(dist_force_data);
    plot_tab->setObserverFeedbackData(obsv_fb_data);
    plot_tab->setMRControllerFeedbackData(mr_ctrl_fb_data);
  }
}

void FlightLogViewerWidget::onPlaybackTimeChanged(double time_from_start)
{
  setPlotData(time_from_start);
}
}  // namespace log
}  // namespace gui
