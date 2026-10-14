// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <filesystem>

#include <rosbag2_cpp/reader.hpp>

#include "./data.hpp"
#include "./playback_control.hpp"
#include "./plot_tab.hpp"

namespace tobas::gui::log
{
class FlightLogViewerWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogViewerWidget;

public:
  explicit FlightLogViewerWidget();

  void reset();

  void setLogName(const QString& _log_name);

private:
  std::filesystem::path log_path_;
  std::unordered_set<std::string> decode_fail_topics_;
  std::vector<std::shared_ptr<DataI>> data_;
  rosbag2_cpp::Reader reader_;

  std::array<PlotTabWidget*, 6> plot_tabs_;
  PlaybackControlWidget* playback_ctrl_;

  template <typename MsgType>
  std::shared_ptr<Data<MsgType>> addData(const std::string& _topic);

  bool open(const std::string& _rosbag_path);

  void setPlotData(double _time_from_start);

private Q_SLOTS:
  void onPlaybackTimeChanged(double _time_from_start);
};

template <typename MsgType>
std::shared_ptr<Data<MsgType>> FlightLogViewerWidget::addData(const std::string& _topic)
{
  const auto res = std::make_shared<Data<MsgType>>(_topic);
  data_.push_back(res);
  return res;
}
}  // namespace tobas::gui::log
