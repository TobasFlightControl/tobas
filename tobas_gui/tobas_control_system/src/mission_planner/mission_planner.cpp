#include "tobas_control_system/mission_planner/mission_planner.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/cast.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_ros2_tools/util.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace gcs
{
MissionPlannerWidget::MissionPlannerWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge)
  : node_(node), mission_thread_(node)
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
  connect(&mission_thread_, &MissionExecutionThread::finished, this, &self::onMissionFinished);
  connect(&bridge, &RosQtBridge::gnssReceived, this, &self::gnssCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::odomReceived, this, &self::odomCb, Qt::QueuedConnection);
}

void MissionPlannerWidget::reset()
{
  map_->setArrowPosition(0., 0.);
  map_->setArrowRotation(0.);

  command_list_->clear();
  commands_->clear();
  pairs_.clear();

  setEditMode();
}

void MissionPlannerWidget::updateNamespace(const std::string& ns)
{
  reset();

  mission_thread_.setNamespace(ns);
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
  auto selected_item = command_list_->selectedItem();

  // 何も選択されていなければ強制的に最初の要素を選択
  if (!selected_item) {
    command_list_->setCurrentRow(0);
    selected_item = command_list_->item(0);
  }

  // 選択アイテムに対応するコマンドを表示
  for (const auto& [item, command] : pairs_) {
    if (item == selected_item) {
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
  const auto selected_item = command_list_->selectedItem();

  for (int i = 0; i < command_list_->count(); ++i) {
    const auto item = command_list_->item(i);
    const auto cmd_type = textToCommand(item->text().toUtf8());
    const auto cmd_widget = getCommandWidget(item);

    switch (cmd_type) {
      case command_t::WAYPOINT: {
        const auto waypoint = qt::qConstPointerCast<WaypointWidget>(cmd_widget);
        const auto latitude = waypoint->latitude();
        const auto longitude = waypoint->longitude();
        const auto coord = QGeoCoordinate(latitude, longitude);

        const auto point_color = item == selected_item ? "orange" : "cyan";
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
      case command_t::TAKEOFF: {
        // TODO
        break;
      }
      case command_t::LAND: {
        // TODO
        break;
      }
      case command_t::RETURN_TO_HOME: {
        // TODO
        break;
      }
      default: {
        throw;
      }
    }
  }
}

BaseCommandWidget* MissionPlannerWidget::getCommandWidget(QListWidgetItem* tar_item)
{
  for (const auto& [item, command] : pairs_) {
    if (item == tar_item) {
      return command;
    }
  }

  throw std::runtime_error("Command widget corresponding to the list item is not found.");
}

QVector<BaseCommandData::SharedPtr> MissionPlannerWidget::createMissionCommandList()
{
  QVector<BaseCommandData::SharedPtr> res;
  for (int i = 0; i < command_list_->count(); ++i) {
    const auto item = command_list_->item(i);
    const auto cmd_widget = getCommandWidget(item);
    res.append(cmd_widget->data());
  }
  return res;
}

void MissionPlannerWidget::onLoadButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onLoadButtonClicked");

  qt::qWarnBox(this, "Not implemented yet.");  // TODO
}

void MissionPlannerWidget::onSaveButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onSaveButtonClicked");

  qt::qWarnBox(this, "Not implemented yet.");  // TODO
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
    case command_t::WAYPOINT: {
      const auto center = map_->getCenter();
      const auto waypoint = new WaypointWidget();
      waypoint->latitude(center.latitude());
      waypoint->longitude(center.longitude());
      cmd_widget = waypoint;
      break;
    }
    case command_t::TAKEOFF: {
      cmd_widget = new TakeoffWidget();
      break;
    }
    case command_t::LAND: {
      cmd_widget = new LandWidget();
      break;
    }
    case command_t::RETURN_TO_HOME: {
      cmd_widget = new ReturnToHomeWidget();
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

  if (!qt::yesOrNo(this, "Do you want to clear all the commands?", qt::QMessageLevel::WARN)) {
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

  if (!qt::yesOrNo(this, "Do you want to cache map tiles to offline storage?", qt::QMessageLevel::WARN)) {
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

  qt::qInfoBox(this, std::format("Map tiles are cached to {}.", kCacheDirOffline).c_str());
}

void MissionPlannerWidget::onExecuteButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onExecuteButtonClicked");

  if (!qt::yesOrNo(this, "Do you want to execute the mission?", qt::QMessageLevel::WARN)) {
    return;
  }

  // ミッションが設定されているかどうかを確認
  if (command_list_->count() == 0) {
    qt::qWarnBox(this, "Mission is empty.");
    return;
  }

  // ミッションデータを抽出
  const auto mission_commands = createMissionCommandList();

  // TODO: 有効なミッションかどうかを確認 (Takeoff後にTakeoffはダメとか)

  // 実行モードに切り替える
  setExecuteMode();

  // ユーザ操作をブロックしないように別スレッドでミッションを実行
  mission_thread_.setCommands(mission_commands);
  mission_thread_.start();
}

void MissionPlannerWidget::onCancelButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onCancelButtonClicked");

  if (!qt::yesOrNo(this, "Do you want to cancel the mission?", qt::QMessageLevel::WARN)) {
    return;
  }

  // ミッションを停止
  mission_thread_.stop();

  // 編集モードに切り替える
  setEditMode();

  qt::qInfoBox(this, "The mission is canceled.");
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

  if (mission_thread_.isRunning()) {
    qt::qWarnBox(this, "You cannot edit the mission while executing it.");
    commandsToMap();
    return;
  }

  int cur_idx = 0;
  for (int i = 0; i < command_list_->count(); ++i) {
    const auto item = command_list_->item(i);
    const auto cmd_type = textToCommand(item->text().toUtf8());
    if (cmd_type == command_t::WAYPOINT) {
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

void MissionPlannerWidget::onMissionFinished(bool success, const QString& message)
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onMissionFinished");

  if (success) {
    qt::qInfoBox(this, "The mission is completed.");
  }
  else {
    qt::qErrorBox(this, message);
  }

  setEditMode();
}

void MissionPlannerWidget::gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss)
{
  if (gnss->fix_type != tobas_msgs::msg::Gnss::FIX_3D) {
    return;
  }

  map_->setArrowPosition(gnss->latitude, gnss->longitude);
}

void MissionPlannerWidget::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  const auto yaw = odom->frame.M.getYaw();
  map_->setArrowRotation(-tobas_std::rad2deg(yaw));
}
}  // namespace gcs
}  // namespace gui
