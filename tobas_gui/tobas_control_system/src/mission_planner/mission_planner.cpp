// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/mission_planner.hpp"

#include <QFileInfo>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <rclcpp_action/create_client.hpp>

#include <tobas_constants/path.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_geographic/geography.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_mission_msgs_adapter/mission.hpp>
#include <tobas_path_tools/core.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/path.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_std_tools/byte.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_trajectory_generation/offline/catmull_rom.hpp>
#include <tobas_yaml_tools/core.hpp>

#include "tobas_control_system/mission_planner/command_type.hpp"
#include "tobas_control_system/mission_planner/save_mission_dialog.hpp"

namespace fs = std::filesystem;

Q_DECLARE_METATYPE(rclcpp_action::ResultCode);

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace
{
using MapSplinePath = traj::CatmullRomPath<Eigen::Vector2d>;

size_t splineMapSampleCount(const MapSplinePath& path, size_t segment)
{
  constexpr double kSplineMapSampleInterval = 1.0;  // [m]
  constexpr size_t kMinSplineMapSamplesPerSegment = 4;
  constexpr size_t kMaxSplineMapSamplesPerSegment = 80;
  constexpr size_t kSplineMapLengthEstimateSamples = 10;

  // This is for display, so approximate the curve length with a small number of points before determining the sample count.
  double length = 0.0;
  auto prev = path.get(segment, 0.0).pos;
  for (size_t sample = 1; sample <= kSplineMapLengthEstimateSamples; ++sample) {
    const auto u = static_cast<double>(sample) / static_cast<double>(kSplineMapLengthEstimateSamples);
    const auto cur = path.get(segment, u).pos;
    length += (cur - prev).norm();
    prev = cur;
  }

  const auto samples = static_cast<size_t>(std::ceil(length / kSplineMapSampleInterval));
  return std::clamp(samples, kMinSplineMapSamplesPerSegment, kMaxSplineMapSamplesPerSegment);
}
}  // namespace

MissionPlannerWidget::MissionPlannerWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
  : node_(node), spinner_(Qt::WindowModal, this)
{
  map_ = new MapWidget();

  load_button_ = new CommandButton("Load");
  save_button_ = new CommandButton("Save");
  add_button_ = new CommandButton("Add");
  clear_button_ = new CommandButton("Clear");
  cache_button_ = new CommandButton("Cache Map");
  execute_button_ = new CommandButton("Execute");
  cancel_button_ = new CommandButton("Cancel");
  focus_button_ = new CommandButton("Focus");

  command_list_ = new qt::ListWidget();
  command_list_->showRowNumber();
  command_list_->setSelectionMode(QListWidget::SingleSelection);
  command_list_->setDragDropMode(QListWidget::InternalMove);

  commands_ = new qt::StackedWidget();

  reset();

  // Layout
  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(load_button_, 1);
  button_cols->addWidget(save_button_, 1);
  button_cols->addWidget(add_button_, 1);
  button_cols->addWidget(clear_button_, 1);
  button_cols->addWidget(cache_button_, 1);
  button_cols->addStretch(0);
  button_cols->addWidget(execute_button_, 1);
  button_cols->addWidget(cancel_button_, 1);
  button_cols->addWidget(focus_button_, 1);

  const auto mission_cols = new QHBoxLayout();
  mission_cols->addWidget(qt::makeGroup("Commands", command_list_), 1);
  mission_cols->addWidget(qt::makeGroup("Command Details", commands_), 3);

  const auto rows = new QVBoxLayout();
  rows->addWidget(map_, 2);
  rows->addLayout(button_cols, 0);
  rows->addLayout(mission_cols, 1);

  setLayout(rows);

  // Connection
  connect(map_, &MapWidget::waypointMoved, this, &self::onWaypointMoved);
  connect(load_button_, &CommandButton::clicked, this, &self::onLoadButtonClicked);
  connect(save_button_, &CommandButton::clicked, this, &self::onSaveButtonClicked);
  connect(add_button_, &CommandButton::clicked, this, &self::onAddButtonClicked);
  connect(clear_button_, &CommandButton::clicked, this, &self::onClearButtonClicked);
  connect(cache_button_, &CommandButton::clicked, this, &self::onCacheButtonClicked);
  connect(execute_button_, &CommandButton::clicked, this, &self::onExecuteButtonClicked);
  connect(cancel_button_, &CommandButton::clicked, this, &self::onCancelButtonClicked);
  connect(focus_button_, &CommandButton::clicked, this, &self::onFocusButtonClicked);
  connect(command_list_, &qt::ListWidget::currentItemChanged, this, &self::onListItemChanged);
  connect(command_list_, &qt::ListWidget::itemMoved, this, &self::onListItemChanged);
  connect(&bridge, &RosQtBridge::gnssReceived, this, &self::gnssCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::odomReceived, this, &self::odomCb, Qt::QueuedConnection);
  connect(this, &self::goalResponseReceived, this, &self::actionGoalResponseCb, Qt::QueuedConnection);
  connect(this, &self::feedbackReceived, this, &self::actionFeedbackCb, Qt::QueuedConnection);
  connect(this, &self::resultReceived, this, &self::actionResultCb, Qt::QueuedConnection);
}

void MissionPlannerWidget::reset()
{
  map_->clear();
  map_->setArrowPosition(0.0, 0.0);
  map_->setArrowRotation(0.0);

  clearMission();
  setEditMode();

  mission_executing_ = false;
}

void MissionPlannerWidget::updateNamespace(const std::string& ns)
{
  reset();

  const auto action_name = path::join(ns, kRemoteIfaceNS, action::kExecuteMission);
  mission_ac_ = rclcpp_action::create_client<Action>(node_, action_name);
}

QString MissionPlannerWidget::getMissionDir()
{
  return settings_store_.value(kLastOpenedDirKey, qt::expandUser(kMissionDir)).toString();
}

void MissionPlannerWidget::setMissionDir(const QString& file_path)
{
  const auto dir = QFileInfo(file_path).absolutePath();
  settings_store_.setValue(kLastOpenedDirKey, dir);
}

void MissionPlannerWidget::clearMission()
{
  command_list_->clear();
  commands_->clear();
  pairs_.clear();
}

void MissionPlannerWidget::addCommand(mission::Type type, BaseCommandWidget* widget)
{
  const auto item = new QListWidgetItem(commandToText(type));
  command_list_->addItem(item);

  commands_->addWidget(widget);
  connect(widget, &BaseCommandWidget::updated, this, &self::onMissionUpdated);
  connect(widget, &BaseCommandWidget::deleteButtonClicked, std::bind(&self::onDeleteButtonClicked, this, item, widget));

  pairs_.insert({ item, widget });
}

void MissionPlannerWidget::setExecuteMode()
{
  load_button_->setEnabled(false);
  save_button_->setEnabled(false);
  add_button_->setEnabled(false);
  clear_button_->setEnabled(false);
  execute_button_->setEnabled(false);
  cancel_button_->setEnabled(true);

  command_list_->setDragDropMode(QListWidget::NoDragDrop);
  commands_->setEnabled(false);
}

void MissionPlannerWidget::setEditMode()
{
  load_button_->setEnabled(true);
  save_button_->setEnabled(true);
  add_button_->setEnabled(true);
  clear_button_->setEnabled(true);
  execute_button_->setEnabled(true);
  cancel_button_->setEnabled(false);

  command_list_->setDragDropMode(QListWidget::InternalMove);
  commands_->setEnabled(true);
}

void MissionPlannerWidget::listToCommands()
{
  if (command_list_->count() == 0) {
    return;
  }

  // Get the selected item.
  auto cur_item = command_list_->currentItem();

  // If nothing is selected, force-select the first item.
  if (!cur_item) {
    command_list_->setCurrentRow(0);
    cur_item = command_list_->item(0);
  }

  // Show the command corresponding to the selected item.
  for (const auto& [item, command] : pairs_) {
    if (item == cur_item) {
      commands_->setCurrentWidget(command);
      return;
    }
  }
  throw std::runtime_error("Failed to find the command widget corresponding to the selected item.");
}

void MissionPlannerWidget::commandsToMap()
{
  updateWaypointSplineEnds();

  map_->clear();

  int wp_index = 1;

  // Draw the `Waypoint` sequence up to the next stop position as one spline.
  std::vector<QGeoCoordinate> spline_waypoints;
  QGeoCoordinate previous_spline_end;
  bool has_previous_spline_end = false;
  const auto cur_item = command_list_->currentItem();

  for (int i = 0; i < command_list_->count(); ++i) {
    const auto item = command_list_->item(i);
    const auto cmd_type = textToCommand(item->text());
    const auto cmd_widget = findCommandWidget(item);

    switch (cmd_type) {
      case mission::Type::kWaypoint: {
        const auto waypoint = qt::qConstPointerCast<WaypointWidget>(cmd_widget);
        const auto latitude = waypoint->latitude();
        const auto longitude = waypoint->longitude();
        const auto coord = QGeoCoordinate(latitude, longitude);

        const auto acceptance_radius = waypoint->isSplineSegmentEnd() ? waypoint->acceptanceRadius() : 0.0;
        const auto point_color = item == cur_item ? "orange" : "cyan";
        map_->addWaypoint(wp_index, coord, acceptance_radius, point_color);

        // If a `Waypoint` follows after a stop, restart the next spline segment from the stop point.
        if (spline_waypoints.empty() && has_previous_spline_end) {
          spline_waypoints.push_back(previous_spline_end);
        }
        spline_waypoints.push_back(coord);

        if (waypoint->isSplineSegmentEnd()) {
          addSplinePathToMap(spline_waypoints);
          previous_spline_end = coord;
          has_previous_spline_end = true;
          spline_waypoints.clear();
        }

        ++wp_index;

        break;
      }
      case mission::Type::kTakeoff:
      case mission::Type::kLand:
      case mission::Type::kReturnToLaunch: {
        // Treat non-`Waypoint` commands as boundaries that split spline sequences.
        addSplinePathToMap(spline_waypoints);
        spline_waypoints.clear();
        has_previous_spline_end = false;
        break;
      }
      default: {
        throw;
      }
    }
  }

  addSplinePathToMap(spline_waypoints);
}

void MissionPlannerWidget::updateWaypointSplineEnds()
{
  for (int i = 0; i < command_list_->count(); ++i) {
    const auto item = command_list_->item(i);
    const auto cmd_type = textToCommand(item->text());
    if (cmd_type != mission::Type::kWaypoint) {
      continue;
    }

    const auto next_is_waypoint =
      i + 1 < command_list_->count() && textToCommand(command_list_->item(i + 1)->text()) == mission::Type::kWaypoint;

    const auto waypoint = qt::qPointerCast<WaypointWidget>(findCommandWidget(item));
    waypoint->setSplineSegmentEnd(waypoint->stopAtWaypoint() || !next_is_waypoint);
  }
}

void MissionPlannerWidget::addSplinePathToMap(const std::vector<QGeoCoordinate>& waypoints)
{
  if (waypoints.size() < 2) {
    return;
  }

  const auto& origin = waypoints.front();
  std::vector<Eigen::Vector2d> points;
  points.reserve(waypoints.size());
  for (const auto& waypoint : waypoints) {
    const auto coord =
      geography_.geodeticToPlane(waypoint.latitude(), waypoint.longitude(), origin.latitude(), origin.longitude());
    points.emplace_back(coord.east, coord.north);
  }

  const MapSplinePath path(std::move(points));

  // Because QML receives a set of short `MapPolyline` objects, approximate the spline with polylines at fixed intervals.
  auto prev_coord = waypoints.front();
  for (size_t segment = 0; segment < path.segmentCount(); ++segment) {
    const auto sample_count = splineMapSampleCount(path, segment);
    for (size_t sample = 1; sample <= sample_count; ++sample) {
      const auto u = static_cast<double>(sample) / static_cast<double>(sample_count);
      const auto pos = path.get(segment, u).pos;
      const auto coord = geography_.planeToGeodetic(pos.x(), pos.y(), origin.latitude(), origin.longitude());
      map_->addLine(prev_coord.latitude(), prev_coord.longitude(), coord.latitude, coord.longitude);
      prev_coord = QGeoCoordinate(coord.latitude, coord.longitude);
    }
  }
}

BaseCommandWidget* MissionPlannerWidget::findCommandWidget(const QListWidgetItem* _item)
{
  for (const auto& [item, command] : pairs_) {
    if (item == _item) {
      return command;
    }
  }
  return nullptr;
}

const BaseCommandWidget* MissionPlannerWidget::findCommandWidget(const QListWidgetItem* _item) const
{
  for (const auto& [item, command] : pairs_) {
    if (item == _item) {
      return command;
    }
  }
  return nullptr;
}

const WaypointWidget* MissionPlannerWidget::findLastWaypoint() const
{
  for (int i = command_list_->count() - 1; i >= 0; --i) {
    const auto list_item = command_list_->item(i);
    const auto cmd_type = textToCommand(list_item->text());
    if (cmd_type == mission::Type::kWaypoint) {
      const auto widget = findCommandWidget(list_item);
      return qt::qConstPointerCast<WaypointWidget>(widget);
    }
  }
  return nullptr;
}

tobas::mission::Mission MissionPlannerWidget::createMission() const
{
  tobas::mission::Mission mission;

  for (int i = 0; i < command_list_->count(); ++i) {
    const auto list_item = command_list_->item(i);
    const auto cmd_type = textToCommand(list_item->text());
    const auto base_widget = findCommandWidget(list_item);

    tobas::mission::MissionItem mission_item;

    switch (cmd_type) {
      case mission::Type::kWaypoint: {
        const auto widget = qt::qConstPointerCast<WaypointWidget>(base_widget);
        mission_item.type = mission::kWaypoint;
        mission_item.data = st::toBytes(widget->dump());
        break;
      }
      case mission::Type::kTakeoff: {
        const auto widget = qt::qConstPointerCast<TakeoffWidget>(base_widget);
        mission_item.type = mission::kTakeoff;
        mission_item.data = st::toBytes(widget->dump());
        break;
      }
      case mission::Type::kLand: {
        const auto widget = qt::qConstPointerCast<LandWidget>(base_widget);
        mission_item.type = mission::kLand;
        mission_item.data = st::toBytes(widget->dump());
        break;
      }
      case mission::Type::kReturnToLaunch: {
        const auto widget = qt::qConstPointerCast<ReturnToLaunchWidget>(base_widget);
        mission_item.type = mission::kReturnToLaunch;
        mission_item.data = st::toBytes(widget->dump());
        break;
      }
      default: {
        throw;
      }
    }

    mission.items.push_back(mission_item);
  }

  return mission;
}

void MissionPlannerWidget::onLoadButtonClicked()
{
  qDebug() << "MissionPlannerWidget::onLoadButtonClicked";

  // Get the mission path.
  const auto dir = getMissionDir();
  const auto file_path = QFileDialog::getOpenFileName(
    this, "Load Mission", dir, "Mission (*.mission);;All Files (*)", nullptr, QFileDialog::DontUseNativeDialog);
  if (file_path.isEmpty()) {
    return;
  }
  setMissionDir(file_path);

  // Load YAML.
  const auto node = yaml::load(file_path.toStdString());
  if (!node) {
    qt::qErrorBox(this, "Failed to load the mission file: " + QString::fromStdString(node.error()));
    return;
  }

  // Parse the mission.
  mission::Mission mission;
  if (!mission.load(node.value())) {
    qt::qErrorBox(this, "Failed to load the mission file.");
    return;
  }

  // Clear the current mission.
  clearMission();

  // Apply the mission to the planner widget.
  for (const auto& [idx, item] : std::views::enumerate(mission.items)) {
    const auto cmd_number = idx + 1;

    switch (item.type) {
      case mission::Type::kWaypoint: {
        mission::Waypoint waypoint;
        if (!st::fromBytes(item.data, waypoint)) {
          qt::qErrorBox(this, "Failed to load mission No. " + QString::number(cmd_number) + ": Waypoint");
          clearMission();
          return;
        }
        const auto widget = new WaypointWidget();
        widget->load(waypoint);
        addCommand(item.type, widget);
        break;
      }
      case mission::Type::kTakeoff: {
        mission::Takeoff takeoff;
        if (!st::fromBytes(item.data, takeoff)) {
          qt::qErrorBox(this, "Failed to load mission No. " + QString::number(cmd_number) + ": Takeoff");
          clearMission();
          return;
        }
        const auto widget = new TakeoffWidget();
        widget->load(takeoff);
        addCommand(item.type, widget);
        break;
      }
      case mission::Type::kLand: {
        mission::Land land;
        if (!st::fromBytes(item.data, land)) {
          qt::qErrorBox(this, "Failed to load mission No. " + QString::number(cmd_number) + ": Land");
          clearMission();
          return;
        }
        const auto widget = new LandWidget();
        widget->load(land);
        addCommand(item.type, widget);
        break;
      }
      case mission::Type::kReturnToLaunch: {
        mission::ReturnToLaunch rtl;
        if (!st::fromBytes(item.data, rtl)) {
          qt::qErrorBox(this, "Failed to load mission No. " + QString::number(cmd_number) + ": ReturnToLaunch");
          clearMission();
          return;
        }
        const auto widget = new ReturnToLaunchWidget();
        widget->load(rtl);
        addCommand(item.type, widget);
        break;
      }
      default: {
        throw;
      }
    }
  }

  // Apply the planner state to the map.
  listToCommands();
  commandsToMap();

  qt::qInfoBox(this, "The mission has been loaded successfully.");
}

void MissionPlannerWidget::onSaveButtonClicked()
{
  qDebug() << "MissionPlannerWidget::onSaveButtonClicked";

  // Check whether a mission exists.
  if (command_list_->count() == 0) {
    qt::qWarnBox(this, "Cannot save an empty mission.");
    return;
  }

  // Get the default directory.
  const auto dir = getMissionDir();

  // Create the directory if it does not exist.
  if (!fs::is_directory(dir.toStdString())) {
    std::error_code ec;
    if (!fs::create_directories(dir.toStdString(), ec)) {
      qt::qErrorBox(this, "Failed to create " + dir + ": " + QString::fromStdString(ec.message()));
      return;
    }
  }

  // Get the mission path.
  SaveMissionDialog dialog(this, dir);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const auto file_path = dialog.selectedFiles().first();
  TOBAS_CHECK(file_path.endsWith(cmn::kMissionExtension));

  // Save the directory opened by the user.
  setMissionDir(file_path);

  // Save the mission.
  const auto mission = createMission();
  const auto node = mission.dump();
  if (!yaml::save(file_path.toStdString(), node)) {
    qt::qErrorBox(this, "Failed to save the current mission: " + file_path);
    return;
  }

  qt::qInfoBox(this, "The current mission has been saved successfully.");
}

void MissionPlannerWidget::onAddButtonClicked()
{
  qDebug() << "MissionPlannerWidget::onAddButtonClicked";

  AddCommandDialog dialog(this);

  const auto res = dialog.exec();
  if (res != QDialog::Accepted) {
    return;
  }

  const auto cmd_type = dialog.selectedCommand();
  BaseCommandWidget* cmd_widget;
  switch (cmd_type) {
    case mission::Type::kWaypoint: {
      const auto new_wp = new WaypointWidget();
      const auto last_wp = findLastWaypoint();
      if (last_wp) {
        // Place the second and later waypoints slightly east of the last point.
        const auto coord = geography_.planeToGeodetic(10.0, 0.0, last_wp->latitude(), last_wp->longitude());
        new_wp->latitude(coord.latitude);
        new_wp->longitude(coord.longitude);
      }
      else {
        // Place the first waypoint at the center of the map.
        const auto center = map_->getCenter();
        new_wp->latitude(center.latitude());
        new_wp->longitude(center.longitude());
      }
      cmd_widget = new_wp;
      break;
    }
    case mission::Type::kTakeoff: {
      cmd_widget = new TakeoffWidget();
      break;
    }
    case mission::Type::kLand: {
      cmd_widget = new LandWidget();
      break;
    }
    case mission::Type::kReturnToLaunch: {
      cmd_widget = new ReturnToLaunchWidget();
      break;
    }
    default: {
      throw;
    }
  }

  addCommand(cmd_type, cmd_widget);
  listToCommands();
  commandsToMap();
}

void MissionPlannerWidget::onClearButtonClicked()
{
  qDebug() << "MissionPlannerWidget::onClearButtonClicked";

  if (!qt::yesOrNo(this, "Do you want to clear all the commands?", qt::WARN)) {
    return;
  }

  map_->clear();
  command_list_->clear();
  commands_->clear();
  pairs_.clear();
}

void MissionPlannerWidget::onCacheButtonClicked()
{
  qDebug() << "MissionPlannerWidget::onCacheButtonClicked";

  if (!qt::yesOrNo(this, "Do you want to cache map tiles to offline storage?", qt::WARN)) {
    return;
  }

  const auto dir_from = qt::expandUser(kCacheDirOnline).toStdString();
  const auto dir_to = qt::expandUser(kCacheDirOffline).toStdString();

  if (!fs::is_directory(dir_to)) {
    TOBAS_CHECK(fs::create_directories(dir_to));
  }

  // Copy all PNG files.
  for (const auto& entry : fs::directory_iterator(dir_from)) {
    if (entry.path().extension() == ".png") {
      const auto& file_from = entry.path();
      const auto file_to = dir_to / file_from.filename();
      fs::copy_file(file_from, file_to, fs::copy_options::overwrite_existing);
    }
  }

  // Get PNG files in the directory.
  std::vector<fs::path> files;
  for (const auto& entry : fs::directory_iterator(dir_to)) {
    if (entry.path().extension() == ".png") {
      files.push_back(entry.path());
    }
  }

  // Sort PNG files by newest modification time first.
  std::sort(
    files.begin(),
    files.end(),
    [](const fs::path& a, const fs::path& b) { return fs::last_write_time(a) > fs::last_write_time(b); });

  // Get file sizes.
  std::vector<uintmax_t> sizes;
  for (const auto& file : files) {
    sizes.push_back(fs::file_size(file));
  }

  // Calculate cumulative file sizes.
  std::vector<uintmax_t> sizes_cs(sizes.size());
  std::partial_sum(sizes.begin(), sizes.end(), sizes_cs.begin());

  // Find the first position that exceeds the maximum size.
  auto it = std::lower_bound(sizes_cs.begin(), sizes_cs.end(), kCacheMaxSize);
  const auto last_alive_idx = std::distance(sizes_cs.begin(), it);

  // Delete files whose total size exceeds the limit.
  for (size_t i = last_alive_idx; i < files.size(); ++i) {
    if (!fs::remove(files[i])) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to remove " << files[i]);
    }
  }

  qt::qInfoBox(this, QString("Map tiles are cached to %1.").arg(kCacheDirOffline));
}

void MissionPlannerWidget::onExecuteButtonClicked()
{
  qDebug() << "MissionPlannerWidget::onExecuteButtonClicked";

  if (!qt::yesOrNo(this, "Do you want to execute the mission?", qt::WARN)) {
    return;
  }

  // Check whether a mission is configured.
  if (command_list_->count() == 0) {
    qt::qWarnBox(this, "Mission is empty.");
    return;
  }

  // Check the mission execution server state.
  if (!mission_ac_->action_server_is_ready()) {
    qt::qWarnBox(this, "Mission executor is not ready.");
    return;
  }

  // Create the mission.
  Action::Goal goal;
  tobas_mission_msgs::MissionAdapter::convert_to_ros_message(createMission(), goal.mission);
  goal.priority.data = tobas_mission_msgs::msg::Priority::NORMAL;

  // Execute the mission.
  Client::SendGoalOptions opts;
  opts.goal_response_callback = [this](const GoalHandle::SharedPtr& gh) { Q_EMIT goalResponseReceived(gh != nullptr); };
  opts.feedback_callback = [this](const GoalHandle::SharedPtr&, const Action::Feedback::ConstSharedPtr& fb)
  { Q_EMIT feedbackReceived(fb->current_command_index); };
  opts.result_callback = [this](const GoalHandle::WrappedResult& res)
  { Q_EMIT resultReceived(res.code, res.result->error_message.c_str(), res.result->last_command_index); };
  mission_ac_->async_send_goal(goal, opts);

  spinner_.start();
}

void MissionPlannerWidget::onCancelButtonClicked()
{
  qDebug() << "MissionPlannerWidget::onCancelButtonClicked";

  if (!qt::yesOrNo(this, "Do you want to cancel the mission?", qt::WARN)) {
    return;
  }

  mission_ac_->async_cancel_all_goals();
}

void MissionPlannerWidget::onFocusButtonClicked()
{
  qDebug() << "MissionPlannerWidget::onFocusButtonClicked";

  const auto arrow_pos = map_->getArrowPosition();
  map_->setMapCenter(arrow_pos.latitude(), arrow_pos.longitude());
}

void MissionPlannerWidget::onDeleteButtonClicked(QListWidgetItem* target_item, BaseCommandWidget* target_widget)
{
  qDebug() << "MissionPlannerWidget::onDeleteButtonClicked";

  command_list_->remove(target_item);
  commands_->removeWidget(target_widget);

  bool found = false;
  for (const auto& pair : pairs_) {
    const auto& item = pair.first;
    const auto& widget = pair.second;
    if (item == target_item && widget == target_widget) {
      found = true;
      pairs_.erase(pair);
      break;
    }
  }
  TOBAS_CHECK(found);

  listToCommands();
  commandsToMap();
}

void MissionPlannerWidget::onListItemChanged()
{
  qDebug() << "MissionPlannerWidget::onListItemChanged";

  listToCommands();
  commandsToMap();
}

void MissionPlannerWidget::onMissionUpdated()
{
  qDebug() << "MissionPlannerWidget::onMissionUpdated";

  commandsToMap();
}

void MissionPlannerWidget::onWaypointMoved(int index, double latitude, double longitude)
{
  qDebug() << "MissionPlannerWidget::onWaypointMoved";

  if (mission_executing_) {
    qt::qWarnBox(this, "You cannot edit the mission while executing it.");
    commandsToMap();
    return;
  }

  int cur_idx = 0;
  for (int i = 0; i < command_list_->count(); ++i) {
    const auto item = command_list_->item(i);
    const auto cmd_type = textToCommand(item->text());
    if (cmd_type == mission::Type::kWaypoint) {
      ++cur_idx;
    }
    if (cur_idx == index) {
      const auto waypoint = qt::qPointerCast<WaypointWidget>(findCommandWidget(item));
      waypoint->latitude(latitude);
      waypoint->longitude(longitude);
      break;
    }
  }

  if (cur_idx != index) {
    throw std::runtime_error(std::format("Index {} is out of range.", index));
  }
}

void MissionPlannerWidget::gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss)
{
  if (gnss->fix_type == tobas_msgs::msg::Gnss::NO_FIX) {
    return;
  }

  map_->setArrowPosition(gnss->latitude, gnss->longitude);
}

void MissionPlannerWidget::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom)
{
  const auto yaw = odom->odom.odom.frame.M.getYaw();

  // Account for the 90 deg offset because east is the heading reference.
  map_->setArrowRotation(-st::rad2deg(yaw - M_PI_2));
}

void MissionPlannerWidget::actionGoalResponseCb(bool ok)
{
  spinner_.stop();

  if (!ok) {
    qt::qErrorBox(this, "The request to execute the mission was rejected. Please see the console messages for details.");
    return;
  }

  mission_executing_ = true;
  setExecuteMode();
}

void MissionPlannerWidget::actionFeedbackCb(uint32_t cur_cmd_idx)
{
  (void)cur_cmd_idx;
  // TODO
}

void MissionPlannerWidget::actionResultCb(rclcpp_action::ResultCode code, const QString& message, uint32_t last_cmd_idx)
{
  switch (code) {
    case rclcpp_action::ResultCode::UNKNOWN:
      qt::qWarnBox(this, "The result of the mission is unknown.");
      break;
    case rclcpp_action::ResultCode::SUCCEEDED:
      qt::qInfoBox(this, "The mission has been completed.");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      qt::qWarnBox(this, "The mission was canceled.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      qt::qErrorBox(
        this,
        "The mission was aborted while executing command No. " + QString::number(last_cmd_idx) + ":\n\n" + message);
      break;
    default:
      qt::qErrorBox(this, "Invalid action result code: " + QString::number((int)code));
      break;
  }

  mission_executing_ = false;
  setEditMode();
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
