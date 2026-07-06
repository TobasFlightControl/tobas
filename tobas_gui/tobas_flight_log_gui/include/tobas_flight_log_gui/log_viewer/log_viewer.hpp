// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

#include <rosbag2_cpp/reader.hpp>

#include "./data.hpp"
#include "./playback_control.hpp"
#include "./plot_tab.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class FlightLogViewerWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogViewerWidget;
  using super = QWidget;

  static constexpr double kWindowDuration = 5.0;  // [s]

public:
  explicit FlightLogViewerWidget();

  void reset();

  void setLogName(const QString& log_name);

private:
  std::filesystem::path log_path_;
  std::unordered_set<std::string> decode_fail_topics_;
  std::vector<std::shared_ptr<DataI>> data_;
  rosbag2_cpp::Reader reader_;

  std::array<PlotTabWidget*, 6> plot_tabs_;
  PlaybackControlWidget* playback_ctrl_;

  template <typename MsgType>
  std::shared_ptr<Data<MsgType>> addData(const std::string& topic);

  bool open(const std::string& rosbag_path);

  void setPlotData(double time_from_start);

private Q_SLOTS:
  void onPlaybackTimeChanged(double time_from_start);
};

template <typename MsgType>
std::shared_ptr<Data<MsgType>> FlightLogViewerWidget::addData(const std::string& topic)
{
  const auto res = std::make_shared<Data<MsgType>>(topic);
  data_.push_back(res);
  return res;
}
}  // namespace log
}  // namespace gui
}  // namespace tobas
