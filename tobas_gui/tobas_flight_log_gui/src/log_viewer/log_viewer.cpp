#include "tobas_flight_log_gui/log_viewer/log_viewer.hpp"

#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_ros2_tools/util.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace log
{
FlightLogViewerWidget::FlightLogViewerWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  for (auto& plot_tab : plot_tabs_) {
    plot_tab = new PlotTabWidget();
    rows->addWidget(plot_tab);
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
  decode_fail_topics_.clear();

  odom_decoder_.clearCache();
  imu_cov_decoder_.clearCache();
  mag_cov_decoder_.clearCache();
  gnss_decoder_.clearCache();
  battery_decoder_.clearCache();
  cur_rotor_states_decoder_.clearCache();
  tar_rotor_speeds_decoder_.clearCache();
  ice_cmd_decoder_.clearCache();
  sampling_time_decoder_.clearCache();
  ctrl_latency_decoder_.clearCache();
  dist_force_decoder_.clearCache();
  obsv_fb_decoder_.clearCache();
  mr_ctrl_fb_decoder_.clearCache();

  for (auto& plot_tab : plot_tabs_) {
    plot_tab->clear();
    plot_tab->setTimeScale(0., kWindowDuration);
  }

  playback_ctrl_->reset();
}

void FlightLogViewerWidget::setLogName(const QString& log_name)
{
  reset();

  // rosbagの絶対パスを更新
  log_path_ = ros2::expandUser(tobas::kRosbagDirHome) / log_name.toStdString();

  // rosbagを開く
  try {
    reader_.open(log_path_);
  }
  catch (const std::exception& e) {
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
  if (log_path_.empty()) {
    qWarning() << "Log path is not set.";
    return;
  }

  if (!fs::exists(log_path_)) {
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
  QVector<tobas_msgs::msg::ImuStamped> raw_imu_data;
  QVector<tobas_msgs::msg::ImuWithCovarianceStamped> filt_imu_data;
  QVector<tobas_msgs::msg::MagneticFieldWithCovarianceStamped> mag_data;
  QVector<tobas_msgs::msg::Gnss> gnss_data;
  QVector<tobas_msgs::msg::Battery> battery_data;
  QVector<tobas_msgs::msg::RotorStateArray> cur_rotor_states_data;
  QVector<tobas_msgs::msg::RotorSpeedArray> tar_rotor_speeds_data;
  QVector<tobas_msgs::msg::IcePropulsionSystemCommand> ice_cmd_data;
  QVector<tobas_msgs::msg::Latency> sampling_time_data;
  QVector<tobas_msgs::msg::Latency> ctrl_latency_data;
  QVector<tobas_kdl_msgs::msg::WrenchStamped> dist_force_data;
  QVector<tobas_debug_msgs::msg::ObserverFeedback> obsv_fb_data;
  QVector<tobas_debug_msgs::msg::MultiRotorControllerFeedback> mr_ctrl_fb_data;
  while (reader_.has_next()) {
    const auto msg = reader_.read_next();

    const auto& cur_time = msg->recv_timestamp;  // [ns]
    if (cur_time > window_stop_time) {
      break;
    }

    // 一度デコードに失敗したトピックはログがリセットされるまでデコードしない
    if (decode_fail_topics_.contains(msg->topic_name)) {
      continue;
    }

    // デコード
    rclcpp::SerializedMessage ser_msg(*msg->serialized_data);
    try {
      if (msg->topic_name.ends_with(path::join("/", tobas::kOdometryTopic))) {
        odom_data.push_back(odom_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kImuRawTopic))) {
        raw_imu_data.push_back(imu_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kImuTopic))) {
        filt_imu_data.push_back(imu_cov_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kMagTopic))) {
        mag_data.push_back(mag_cov_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kGnssTopic))) {
        gnss_data.push_back(gnss_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kBatteryTopic))) {
        battery_data.push_back(battery_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kRotorStatesTopic))) {
        cur_rotor_states_data.push_back(cur_rotor_states_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kRotorSpeedsCmdTopic))) {
        tar_rotor_speeds_data.push_back(tar_rotor_speeds_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kIcePropulsionSystemCmdTopic))) {
        ice_cmd_data.push_back(ice_cmd_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kImuSamplingTimeTopic))) {
        sampling_time_data.push_back(sampling_time_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kControlLatencyTopic))) {
        ctrl_latency_data.push_back(ctrl_latency_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kDisturbanceForceTopic))) {
        dist_force_data.push_back(dist_force_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kObsvFeedbackTopic))) {
        obsv_fb_data.push_back(obsv_fb_decoder_.decode(cur_time, ser_msg));
      }
      else if (msg->topic_name.ends_with(path::join("/", tobas::kMRCtrlFeedbackTopic))) {
        mr_ctrl_fb_data.push_back(mr_ctrl_fb_decoder_.decode(cur_time, ser_msg));
      }
    }
    catch (const std::exception& e) {
      qt::qErrorBox(this, "Failed to deserialize \"" + QString::fromStdString(msg->topic_name) + "\".");
      decode_fail_topics_.insert(msg->topic_name);
    }
  }

  // データをプロット
  for (auto& plot_tab : plot_tabs_) {
    // XXX: データの設定の前に範囲を指定しないと若干プロットが崩れる
    plot_tab->setTimeScale(window_start_time * 1e-9, window_stop_time * 1e-9);

    plot_tab->setFrameData(odom_data, mr_ctrl_fb_data);
    plot_tab->setImuData(raw_imu_data, filt_imu_data);
    plot_tab->setMagData(mag_data);
    plot_tab->setGnssData(gnss_data);
    plot_tab->setBatteryData(battery_data);
    plot_tab->setEngineData(ice_cmd_data);
    plot_tab->setRotorSpeedData(cur_rotor_states_data, tar_rotor_speeds_data);
    plot_tab->setPropellerPitchData(ice_cmd_data);
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
