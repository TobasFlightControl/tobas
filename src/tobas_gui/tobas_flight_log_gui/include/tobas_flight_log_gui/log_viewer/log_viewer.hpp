// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>
#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <QFrame>
#include <QGridLayout>
#include <QVector>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include "./data.hpp"
#include "./playback_control.hpp"
#include "./plot_panel.hpp"

namespace tobas::gui::log
{
class FlightLogViewerWidget : public QWidget
{
  Q_OBJECT

  using self = FlightLogViewerWidget;
  using Time = rcutils_time_point_value_t;
  using BagMessagePtr = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;

public:
  explicit FlightLogViewerWidget();

  void reset();

  void setLogName(const QString& _log_name);

private:
  std::filesystem::path log_path_;
  rosbag2_cpp::Reader reader_;
  std::vector<std::unique_ptr<DataI>> data_;
  std::unordered_map<std::string, DataI*> topic_data_;  ///< Failed topics map to `nullptr` until reset.
  std::deque<BagMessagePtr> window_messages_;
  Time window_start_time_, window_stop_time_;

  std::array<PlotPanelWidget*, 6> plot_panels_;
  std::array<QFrame*, 3> plot_separators_;
  QGridLayout* plot_grid_;
  PlaybackControlWidget* playback_ctrl_;

  /** Register a stream and return its values, which remain valid for the viewer's lifetime. */
  template <typename MsgType>
  const QVector<MsgType>& addData(const std::string& _topic);

  bool open(const std::string& _rosbag_path);

  void readWindow(Time _start, Time _stop);

  void setPlotData(double _time_from_start);

  void plotPanel(PlotPanelWidget* _panel);

  void setExpandedPanel(PlotPanelWidget* _expanded_panel);

private Q_SLOTS:
  void onPlaybackTimeChanged(double _time_from_start);
};

template <typename MsgType>
const QVector<MsgType>& FlightLogViewerWidget::addData(const std::string& _topic)
{
  auto stream = std::make_unique<Data<MsgType>>(_topic);
  const auto& values = stream->getValues();
  data_.push_back(std::move(stream));
  return values;
}
}  // namespace tobas::gui::log
