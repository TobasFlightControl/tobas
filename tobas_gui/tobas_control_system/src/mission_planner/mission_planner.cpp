// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/mission_planner.hpp"

#include <ranges>

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/path.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_mission_msgs_adapter/mission.hpp>
#include <tobas_path_tools/core.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/path.hpp>
#include <tobas_std_tools/byte.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/core.hpp>

#include "tobas_control_system/mission_planner/command_type.hpp"
#include "tobas_control_system/mission_planner/commands/commands.hpp"
#include "tobas_control_system/mission_planner/save_mission_dialog.hpp"

namespace fs = std::filesystem;

Q_DECLARE_METATYPE(rclcpp_action::ResultCode);

namespace tobas
{
namespace gui
{
namespace ctrl
{
MissionPlannerWidget::MissionPlannerWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
  : node_(node), property_client_(node, "tobas_control_system/mission_planner"), spinner_(Qt::WindowModal, this)
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
  commands_->setStyleSheet("QStackedWidget { border: 1px solid black; background-color: white; }");

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
  mission_cols->addWidget(command_list_);
  mission_cols->addWidget(commands_);

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
  connect(command_list_, &qt::ListWidget::itemClicked, this, &self::onListItemChanged);
  connect(command_list_, &qt::ListWidget::itemMoved, this, &self::onListItemChanged);
  connect(&bridge, &RosQtBridge::gnssReceived, this, &self::gnssCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::odomReceived, this, &self::odomCb, Qt::QueuedConnection);
  connect(this, &self::goalResponseReceived, this, &self::actionGoalResponseCb, Qt::QueuedConnection);
  connect(this, &self::resultReceived, this, &self::actionResultCb, Qt::QueuedConnection);
}

void MissionPlannerWidget::reset()
{
  map_->clear();
  map_->setArrowPosition(0., 0.);
  map_->setArrowRotation(0.);

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
  std::string last_opened_dir;
  if (property_client_.get(kLastOpenedDirKey, last_opened_dir) == ptree::PropertyClient::kNoError) {
    return QString::fromStdString(last_opened_dir);
  }
  else {
    qWarning() << property_client_.errorMessage();
    return qt::expandUser(kMissionDir);
  }
}

void MissionPlannerWidget::setMissionDir(const QString& file_path)
{
  fs::path p(file_path.toStdString());
  const auto dir = p.parent_path().string();

  if (property_client_.set(kLastOpenedDirKey, dir) < 0) {
    qWarning() << property_client_.errorMessage();
    return;
  }
  if (property_client_.save() < 0) {
    qWarning() << property_client_.errorMessage();
    return;
  }
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

  // 選択されているアイテムを取得
  auto cur_item = command_list_->currentItem();

  // 何も選択されていなければ強制的に最初の要素を選択
  if (!cur_item) {
    command_list_->setCurrentRow(0);
    cur_item = command_list_->item(0);
  }

  // 選択アイテムに対応するコマンドを表示
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
  map_->clear();

  int index = 1;
  bool is_first_waypoint = true;
  QGeoCoordinate last_coord;
  const auto cur_item = command_list_->currentItem();

  for (int i = 0; i < command_list_->count(); ++i) {
    const auto item = command_list_->item(i);
    const auto cmd_type = textToCommand(item->text().toUtf8());
    const auto cmd_widget = getCommandWidget(item);

    switch (cmd_type) {
      case mission::Type::kWaypoint: {
        const auto waypoint = qt::qConstPointerCast<WaypointWidget>(cmd_widget);
        const auto latitude = waypoint->latitude();
        const auto longitude = waypoint->longitude();
        const auto coord = QGeoCoordinate(latitude, longitude);

        const auto point_color = item == cur_item ? "orange" : "cyan";
        map_->addWaypoint(index, coord, waypoint->acceptanceRadius(), point_color);

        if (is_first_waypoint) {
          is_first_waypoint = false;
        }
        else {
          map_->addLine(last_coord.latitude(), last_coord.longitude(), latitude, longitude);
        }

        ++index;
        last_coord.setLatitude(latitude);
        last_coord.setLongitude(longitude);

        break;
      }
      case mission::Type::kTakeoff: {
        break;
      }
      case mission::Type::kLand: {
        break;
      }
      case mission::Type::kReturnToLaunch: {
        break;
      }
      default: {
        throw;
      }
    }
  }
}

BaseCommandWidget* MissionPlannerWidget::getCommandWidget(QListWidgetItem* tar_item) const
{
  for (const auto& [item, command] : pairs_) {
    if (item == tar_item) {
      return command;
    }
  }

  throw std::runtime_error("Command widget corresponding to the list item is not found.");
}

tobas::mission::Mission MissionPlannerWidget::createMission() const
{
  tobas::mission::Mission mission;

  for (int i = 0; i < command_list_->count(); ++i) {
    const auto list_item = command_list_->item(i);
    const auto cmd_type = textToCommand(list_item->text().toUtf8());
    const auto base_widget = getCommandWidget(list_item);

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

  // ミッションのパスを取得
  const auto dir = getMissionDir();
  const auto file_path = QFileDialog::getOpenFileName(
    this, "Load Mission", dir, "Mission (*.mission);;All Files (*)", nullptr, QFileDialog::DontUseNativeDialog);
  if (file_path.isEmpty()) {
    return;
  }
  setMissionDir(file_path);

  // YAMLを読み込む
  const auto node = yaml::load(file_path.toStdString());
  if (!node) {
    qt::qErrorBox(this, "Failed to load the mission file: " + QString::fromStdString(node.error()));
    return;
  }

  // ミッションを解析
  mission::Mission mission;
  if (!mission.load(node.value())) {
    qt::qErrorBox(this, "Failed to load the mission file.");
    return;
  }

  // 現在のミッションを消去
  clearMission();

  // ミッションをプランナーウィジェットに反映
  for (const auto& [idx, item] : std::views::enumerate(mission.items)) {
    switch (item.type) {
      case mission::Type::kWaypoint: {
        mission::Waypoint waypoint;
        if (!st::fromBytes(item.data, waypoint)) {
          qt::qErrorBox(this, "Failed to load mission No. " + QString::number(idx) + ": Waypoint");
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
          qt::qErrorBox(this, "Failed to load mission No. " + QString::number(idx) + ": Takeoff");
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
          qt::qErrorBox(this, "Failed to load mission No. " + QString::number(idx) + ": Land");
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
          qt::qErrorBox(this, "Failed to load mission No. " + QString::number(idx) + ": ReturnToLaunch");
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

  // プランナーの状態をマップに反映
  listToCommands();
  commandsToMap();

  qt::qInfoBox(this, "The mission has been loaded successfully.");
}

void MissionPlannerWidget::onSaveButtonClicked()
{
  qDebug() << "MissionPlannerWidget::onSaveButtonClicked";

  // ミッションが存在するか確認
  if (command_list_->count() == 0) {
    qt::qWarnBox(this, "Cannot save an empty mission.");
    return;
  }

  // デフォルトのディレクトリを取得
  const auto dir = getMissionDir();

  // ディレクトリが存在しなければ作成
  if (!fs::is_directory(dir.toStdString())) {
    std::error_code ec;
    if (!fs::create_directories(dir.toStdString(), ec)) {
      qt::qErrorBox(this, "Failed to create " + dir + ": " + QString::fromStdString(ec.message()));
      return;
    }
  }

  // ミッションのパスを取得
  SaveMissionDialog dialog(this, dir);
  if (dialog.exec() != QDialog::Accepted) {
    return;
  }
  const auto file_path = dialog.selectedFiles().first();
  TOBAS_CHECK(file_path.endsWith(cmn::kMissionExtension));

  // ユーザが開いたディレクトリを保存
  setMissionDir(file_path);

  // ミッションを保存
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
      const auto center = map_->getCenter();
      const auto waypoint = new WaypointWidget();
      waypoint->latitude(center.latitude());
      waypoint->longitude(center.longitude());
      cmd_widget = waypoint;
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

  // 全てのPNGファイルをコピー
  for (const auto& entry : fs::directory_iterator(dir_from)) {
    if (entry.path().extension() == ".png") {
      const auto& file_from = entry.path();
      const auto file_to = dir_to / file_from.filename();
      fs::copy_file(file_from, file_to, fs::copy_options::overwrite_existing);
    }
  }

  // ディレクトリ内のPNGファイルを取得
  std::vector<fs::path> files;
  for (const auto& entry : fs::directory_iterator(dir_to)) {
    if (entry.path().extension() == ".png") {
      files.push_back(entry.path());
    }
  }

  // PNGファイルを最終変更時刻が新しい順にソート
  std::sort(
    files.begin(),
    files.end(),
    [](const fs::path& a, const fs::path& b) { return fs::last_write_time(a) > fs::last_write_time(b); });

  // ファイルサイズを取得
  std::vector<uintmax_t> sizes;
  for (const auto& file : files) {
    sizes.push_back(fs::file_size(file));
  }

  // ファイルサイズの累積和を計算
  std::vector<uintmax_t> sizes_cs(sizes.size());
  std::partial_sum(sizes.begin(), sizes.end(), sizes_cs.begin());

  // 最大サイズを超える最初の位置を見つける
  auto it = std::lower_bound(sizes_cs.begin(), sizes_cs.end(), kCacheMaxSize);
  const auto last_alive_idx = std::distance(sizes_cs.begin(), it);

  // サイズがリミットを超えたファイルを削除
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

  // ミッションが設定されているかどうかを確認
  if (command_list_->count() == 0) {
    qt::qWarnBox(this, "Mission is empty.");
    return;
  }

  // ミッション実行サーバの状態を確認
  if (!mission_ac_->action_server_is_ready()) {
    qt::qWarnBox(this, "Mission executor is not ready.");
    return;
  }

  // ミッションを作成
  Action::Goal goal;
  tobas_mission_msgs::MissionAdapter::convert_to_ros_message(createMission(), goal.mission);
  goal.priority.data = tobas_mission_msgs::msg::Priority::NORMAL;

  // ミッションを実行
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
    const auto cmd_type = textToCommand(item->text().toUtf8());
    if (cmd_type == mission::Type::kWaypoint) {
      ++cur_idx;
    }
    if (cur_idx == index) {
      const auto waypoint = qt::qPointerCast<WaypointWidget>(getCommandWidget(item));
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
  map_->setArrowRotation(-st::rad2deg(yaw - M_PI_2));  // 東向きが方位の基準なので90degのオフセットを考慮
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
