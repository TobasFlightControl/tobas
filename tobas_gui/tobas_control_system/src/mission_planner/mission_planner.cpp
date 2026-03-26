#include "tobas_control_system/mission_planner/mission_planner.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_mission_items/mission_items.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/byte.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

#include "tobas_control_system/mission_planner/command_type.hpp"
#include "tobas_control_system/mission_planner/commands/commands.hpp"

namespace fs = std::filesystem;

Q_DECLARE_METATYPE(rclcpp_action::ResultCode);

namespace tobas
{
namespace gui
{
namespace ctrl
{
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

  command_list_ = new tobas::qt::ListWidget();
  command_list_->setSelectionMode(QListWidget::SingleSelection);
  command_list_->setDragDropMode(QListWidget::InternalMove);

  commands_ = new tobas::qt::StackedWidget();
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
  connect(command_list_, &tobas::qt::ListWidget::itemClicked, this, &self::onListItemChanged);
  connect(command_list_, &tobas::qt::ListWidget::itemMoved, this, &self::onListItemChanged);
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

  command_list_->clear();
  commands_->clear();
  pairs_.clear();

  mission_executing_ = false;

  setEditMode();
}

void MissionPlannerWidget::updateNamespace(const std::string& ns)
{
  reset();

  const auto action_name = path::join(ns, tobas::kRemoteIfaceNS, action::kExecuteMission);
  mission_ac_ = rclcpp_action::create_client<Action>(node_, action_name);
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
      case Command::kWaypoint: {
        const auto waypoint = tobas::qt::qConstPointerCast<WaypointWidget>(cmd_widget);
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
      case Command::kTakeoff: {
        // TODO
        break;
      }
      case Command::kLand: {
        // TODO
        break;
      }
      case Command::kReturnToLaunch: {
        // TODO
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

MissionPlannerWidget::Action::Goal MissionPlannerWidget::createMissionGoal() const
{
  Action::Goal goal;
  goal.priority.data = tobas_mission_msgs::msg::Priority::NORMAL;

  for (int i = 0; i < command_list_->count(); ++i) {
    const auto list_item = command_list_->item(i);
    const auto cmd_type = textToCommand(list_item->text().toUtf8());
    const auto base_widget = getCommandWidget(list_item);

    tobas_mission_msgs::msg::MissionItem mission_item;

    switch (cmd_type) {
      case Command::kWaypoint: {
        const auto widget = tobas::qt::qConstPointerCast<WaypointWidget>(base_widget);

        tobas::mission::Waypoint waypoint;
        waypoint.latitude = widget->latitude();
        waypoint.longitude = widget->longitude();
        waypoint.altitude = widget->altitude();
        waypoint.altitude_frame = widget->altitudeFrame();
        waypoint.auto_heading = true;  // TODO
        waypoint.max_horizontal_velocity = widget->maxHorizontalVelocity();
        waypoint.max_horizontal_accel = widget->maxHorizontalAccel();
        waypoint.max_horizontal_jerk = widget->maxHorizontalJerk();
        waypoint.max_vertical_velocity = widget->maxVerticalVelocity();
        waypoint.max_vertical_accel = widget->maxVerticalAccel();
        waypoint.max_vertical_jerk = widget->maxVerticalJerk();
        waypoint.max_heading_rate = widget->maxHeadingRate();
        waypoint.max_heading_accel = widget->maxHeadingAccel();
        waypoint.acceptance_radius = widget->acceptanceRadius();
        waypoint.altitude_tolerance = widget->altitudeTolerance();
        waypoint.timeout = 0.;  // TODO

        mission_item.type = tobas::mission::kWaypoint;
        mission_item.data = st::toBytes(waypoint);

        break;
      }
      case Command::kTakeoff: {
        const auto widget = tobas::qt::qConstPointerCast<TakeoffWidget>(base_widget);

        tobas::mission::Takeoff takeoff;
        takeoff.altitude = widget->altitude();
        takeoff.altitude_frame = widget->altitudeFrame();
        takeoff.max_speed = widget->maxSpeed();
        takeoff.max_accel = widget->maxAccel();
        takeoff.max_jerk = widget->maxJerk();
        takeoff.altitude_tolerance = widget->altitudeTolerance();
        takeoff.timeout = 0.;  // TODO

        mission_item.type = tobas::mission::kTakeoff;
        mission_item.data = st::toBytes(takeoff);

        break;
      }
      case Command::kLand: {
        const auto widget = tobas::qt::qConstPointerCast<LandWidget>(base_widget);

        tobas::mission::Land land;
        land.speed = widget->speed();
        land.timeout = 0.;  // TODO

        mission_item.type = tobas::mission::kLand;
        mission_item.data = st::toBytes(land);

        break;
      }
      case Command::kReturnToLaunch: {
        const auto widget = tobas::qt::qConstPointerCast<ReturnToLaunchWidget>(base_widget);

        tobas::mission::ReturnToLaunch rtl;
        rtl.min_altitude = widget->minAltitude();
        rtl.max_horizontal_velocity = widget->maxHorizontalVelocity();
        rtl.max_horizontal_accel = widget->maxHorizontalAccel();
        rtl.max_horizontal_jerk = widget->maxHorizontalJerk();
        rtl.max_vertical_velocity = widget->maxVerticalVelocity();
        rtl.max_vertical_accel = widget->maxVerticalAccel();
        rtl.max_vertical_jerk = widget->maxVerticalJerk();
        rtl.max_heading_rate = widget->maxHeadingRate();
        rtl.max_heading_accel = widget->maxHeadingAccel();
        rtl.acceptance_radius = widget->acceptanceRadius();
        rtl.altitude_tolerance = widget->altitudeTolerance();
        rtl.timeout = 0.;  // TODO

        mission_item.type = tobas::mission::kReturnToLaunch;
        mission_item.data = st::toBytes(rtl);

        break;
      }
      default: {
        throw;
      }
    }

    goal.items.push_back(mission_item);
  }

  return goal;
}

void MissionPlannerWidget::onLoadButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onLoadButtonClicked");

  tobas::qt::qWarnBox(this, "Not implemented yet.");  // TODO
}

void MissionPlannerWidget::onSaveButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onSaveButtonClicked");

  tobas::qt::qWarnBox(this, "Not implemented yet.");  // TODO
}

void MissionPlannerWidget::onAddButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onAddButtonClicked");

  AddCommandDialog dialog(this);

  const auto res = dialog.exec();
  if (res != QDialog::Accepted) {
    return;
  }

  const auto cmd_type = dialog.selectedCommand();
  BaseCommandWidget* cmd_widget;
  switch (cmd_type) {
    case Command::kWaypoint: {
      const auto center = map_->getCenter();
      const auto waypoint = new WaypointWidget();
      waypoint->latitude(center.latitude());
      waypoint->longitude(center.longitude());
      cmd_widget = waypoint;
      break;
    }
    case Command::kTakeoff: {
      cmd_widget = new TakeoffWidget();
      break;
    }
    case Command::kLand: {
      cmd_widget = new LandWidget();
      break;
    }
    case Command::kReturnToLaunch: {
      cmd_widget = new ReturnToLaunchWidget();
      break;
    }
    default: {
      throw;
    }
  }

  const auto item = new QListWidgetItem(commandToText(cmd_type));
  command_list_->addItem(item);

  commands_->addWidget(cmd_widget);
  connect(cmd_widget, &BaseCommandWidget::updated, this, &self::onMissionUpdated);
  connect(
    cmd_widget,
    &BaseCommandWidget::deleteButtonClicked,
    std::bind(&self::onDeleteButtonClicked, this, item, cmd_widget));

  pairs_.insert({ item, cmd_widget });

  listToCommands();
  commandsToMap();
}

void MissionPlannerWidget::onClearButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onClearButtonClicked");

  if (!tobas::qt::yesOrNo(this, "Do you want to clear all the commands?", tobas::qt::WARN)) {
    return;
  }

  map_->clear();
  command_list_->clear();
  commands_->clear();
  pairs_.clear();
}

void MissionPlannerWidget::onCacheButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onCacheButtonClicked");

  if (!tobas::qt::yesOrNo(this, "Do you want to cache map tiles to offline storage?", tobas::qt::WARN)) {
    return;
  }

  const auto dir_from = ros2::expandUser(kCacheDirOnline);
  const auto dir_to = ros2::expandUser(kCacheDirOffline);

  if (!fs::is_directory(dir_to)) {
    fs::create_directories(dir_to);
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

  tobas::qt::qInfoBox(this, QString("Map tiles are cached to %1.").arg(kCacheDirOffline));
}

void MissionPlannerWidget::onExecuteButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onExecuteButtonClicked");

  if (!tobas::qt::yesOrNo(this, "Do you want to execute the mission?", tobas::qt::WARN)) {
    return;
  }

  // ミッションが設定されているかどうかを確認
  if (command_list_->count() == 0) {
    tobas::qt::qWarnBox(this, "Mission is empty.");
    return;
  }

  // ミッション実行サーバの状態を確認
  if (!mission_ac_->action_server_is_ready()) {
    tobas::qt::qWarnBox(this, "Mission executor is not ready.");
    return;
  }

  // ミッションを実行
  const auto goal = createMissionGoal();
  Client::SendGoalOptions opts;
  opts.goal_response_callback = [this](const GoalHandle::SharedPtr& gh) { Q_EMIT goalResponseReceived(gh != nullptr); };
  opts.feedback_callback = [this](const GoalHandle::SharedPtr&, const Action::Feedback::ConstSharedPtr& fb)
  { Q_EMIT feedbackReceived(fb->current_index); };
  opts.result_callback = [this](const GoalHandle::WrappedResult& res)
  { Q_EMIT resultReceived(res.code, QString::fromStdString(res.result->error_message)); };
  mission_ac_->async_send_goal(goal, opts);

  spinner_.start();
}

void MissionPlannerWidget::onCancelButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onCancelButtonClicked");

  if (!tobas::qt::yesOrNo(this, "Do you want to cancel the mission?", tobas::qt::WARN)) {
    return;
  }

  mission_ac_->async_cancel_all_goals();
}

void MissionPlannerWidget::onFocusButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onFocusButtonClicked");

  const auto arrow_pos = map_->getArrowPosition();
  map_->setMapCenter(arrow_pos.latitude(), arrow_pos.longitude());
}

void MissionPlannerWidget::onDeleteButtonClicked(QListWidgetItem* target_item, BaseCommandWidget* target_widget)
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onDeleteButtonClicked");

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
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onListItemChanged");

  listToCommands();
  commandsToMap();
}

void MissionPlannerWidget::onMissionUpdated()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onMissionUpdated");

  commandsToMap();
}

void MissionPlannerWidget::onWaypointMoved(int index, double latitude, double longitude)
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onWaypointMoved");

  if (mission_executing_) {
    tobas::qt::qWarnBox(this, "You cannot edit the mission while executing it.");
    commandsToMap();
    return;
  }

  int cur_idx = 0;
  for (int i = 0; i < command_list_->count(); ++i) {
    const auto item = command_list_->item(i);
    const auto cmd_type = textToCommand(item->text().toUtf8());
    if (cmd_type == Command::kWaypoint) {
      ++cur_idx;
    }
    if (cur_idx == index) {
      const auto waypoint = tobas::qt::qPointerCast<WaypointWidget>(getCommandWidget(item));
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
    tobas::qt::qErrorBox(this, "The request to execute the mission was rejected.");
    return;
  }

  mission_executing_ = true;
  setExecuteMode();
}

void MissionPlannerWidget::actionFeedbackCb(uint32_t current_index)
{
  (void)current_index;
  // TODO
}

void MissionPlannerWidget::actionResultCb(rclcpp_action::ResultCode code, const QString& message)
{
  switch (code) {
    case rclcpp_action::ResultCode::UNKNOWN:
      tobas::qt::qWarnBox(this, "The result of the mission is unknown.");
      break;
    case rclcpp_action::ResultCode::SUCCEEDED:
      tobas::qt::qInfoBox(this, "The mission has been completed.");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      tobas::qt::qWarnBox(this, "The mission was canceled.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      tobas::qt::qErrorBox(this, "The mission was aborted:\n\n" + message);
      break;
    default:
      tobas::qt::qErrorBox(this, "Invalid action result code: " + QString::number((int)code));
      break;
  }

  mission_executing_ = false;
  setEditMode();
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
