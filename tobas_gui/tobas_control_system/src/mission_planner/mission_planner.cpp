#include <QVBoxLayout>
#include <QHBoxLayout>

#include <tobas_std_tools/check.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_qt_tools/message.hpp>

#include "tobas_control_system/mission_planner/mission_planner.hpp"

namespace fs = std::filesystem;

namespace gui
{
namespace control_system
{
MissionPlannerWidget::MissionPlannerWidget(rclcpp::Node::SharedPtr node) : node_(node), mission_thread_(node)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  map_ = new MapWidget();
  rows->addWidget(map_);

  const auto button_cols = new QHBoxLayout();
  rows->addLayout(button_cols);

  load_button_ = new QPushButton("Load");
  load_button_->setFixedSize(kButtonWidth, kButtonHeight);
  button_cols->addWidget(load_button_);

  save_button_ = new QPushButton("Save");
  save_button_->setFixedSize(kButtonWidth, kButtonHeight);
  button_cols->addWidget(save_button_);

  add_button_ = new QPushButton("Add");
  add_button_->setFixedSize(kButtonWidth, kButtonHeight);
  button_cols->addWidget(add_button_);

  clear_button_ = new QPushButton("Clear");
  clear_button_->setFixedSize(kButtonWidth, kButtonHeight);
  button_cols->addWidget(clear_button_);

  cache_button_ = new QPushButton("Cache Map");
  cache_button_->setFixedSize(kButtonWidth, kButtonHeight);
  button_cols->addWidget(cache_button_);

  button_cols->addStretch();

  execute_button_ = new QPushButton("Execute");
  execute_button_->setFixedSize(kButtonWidth, kButtonHeight);
  button_cols->addWidget(execute_button_);

  cancel_button_ = new QPushButton("Cancel");
  cancel_button_->setFixedSize(kButtonWidth, kButtonHeight);
  button_cols->addWidget(cancel_button_);

  focus_button_ = new QPushButton("Focus");
  focus_button_->setFixedSize(kButtonWidth, kButtonHeight);
  button_cols->addWidget(focus_button_);

  const auto mission_cols = new QHBoxLayout();
  rows->addLayout(mission_cols);

  command_list_ = new qt::ListWidget();
  command_list_->setSelectionMode(QListWidget::SingleSelection);
  command_list_->setDragDropMode(QListWidget::InternalMove);
  mission_cols->addWidget(command_list_);

  commands_ = new qt::StackedWidget();
  commands_->setStyleSheet("QStackedWidget { border: 1px solid black; background-color: white; }");
  mission_cols->addWidget(commands_);

  setEditMode();
  setEnabled(false);

  // Connections
  connect(map_, &MapWidget::waypointMoved, this, &self::onWaypointMoved);
  connect(load_button_, &QPushButton::clicked, this, &self::onLoadButtonClicked);
  connect(save_button_, &QPushButton::clicked, this, &self::onSaveButtonClicked);
  connect(add_button_, &QPushButton::clicked, this, &self::onAddButtonClicked);
  connect(clear_button_, &QPushButton::clicked, this, &self::onClearButtonClicked);
  connect(cache_button_, &QPushButton::clicked, this, &self::onCacheButtonClicked);
  connect(execute_button_, &QPushButton::clicked, this, &self::onExecuteButtonClicked);
  connect(cancel_button_, &QPushButton::clicked, this, &self::onCancelButtonClicked);
  connect(focus_button_, &QPushButton::clicked, this, &self::onFocusButtonClicked);
  connect(command_list_, &qt::ListWidget::itemClicked, this, &self::onListItemChanged);
  connect(command_list_, &qt::ListWidget::itemMoved, this, &self::onListItemChanged);
  connect(&mission_thread_, &MissionExecutionThread::finished, this, &self::onMissionFinished);
}

void MissionPlannerWidget::updateNamespace(const std::string& ns)
{
  ns_ = ns;

  gps_ = nullptr;
  gps_sub_ = ros2::createSubscriber(node_, path::join(ns, tobas::kGpsTopic), &self::gpsCb, this);

  setEnabled(true);
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
  if (command_list_->count() == 0)
    return;

  // 選択されているアイテムを取得
  auto selected_item = command_list_->selectedItem();

  // 何も選択されていなければ強制的に最初の要素を選択
  if (selected_item == nullptr)
  {
    command_list_->setCurrentRow(0);
    selected_item = command_list_->item(0);
  }

  // 選択アイテムに対応するコマンドを表示
  for (const auto& [item, command] : pairs_)
  {
    if (item == selected_item)
    {
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

  for (int i = 0; i < command_list_->count(); ++i)
  {
    const auto item = command_list_->item(i);
    const auto cmd_type = textToCommand(item->text().toUtf8());
    const auto cmd_widget = getCommandWidget(item);

    switch (cmd_type)
    {
      case command_t::WAYPOINT:
      {
        const auto waypoint = qobject_cast<WaypointWidget*>(cmd_widget);
        const auto latitude = waypoint->latitude();
        const auto longitude = waypoint->longitude();
        const auto coord = QGeoCoordinate(latitude, longitude);

        const auto point_color = item == selected_item ? "orange" : "cyan";
        map_->addWaypoint(index, coord, waypoint->acceptanceRadius(), point_color);

        if (is_first_waypoint)
          is_first_waypoint = false;
        else
          map_->addLine(last_coord.latitude(), last_coord.longitude(), latitude, longitude);

        ++index;
        last_coord.setLatitude(latitude);
        last_coord.setLongitude(longitude);

        break;
      }
      case command_t::TAKEOFF:
      {
        // TODO
        break;
      }
      case command_t::LAND:
      {
        // TODO
        break;
      }
      case command_t::RETURN_TO_HOME:
      {
        // TODO
        break;
      }
      default:
      {
        throw;
      }
    }
  }
}

BaseCommandWidget* MissionPlannerWidget::getCommandWidget(QListWidgetItem* tar_item)
{
  for (const auto& [item, command] : pairs_)
    if (item == tar_item)
      return command;

  throw std::runtime_error("Command widget corresponding to the list item is not found.");
}

QVector<BaseCommandData::SharedPtr> MissionPlannerWidget::createMissionCommandList()
{
  QVector<BaseCommandData::SharedPtr> res;
  for (int i = 0; i < command_list_->count(); ++i)
  {
    const auto item = command_list_->item(i);
    const auto cmd_widget = getCommandWidget(item);
    res.append(cmd_widget->data());
  }
  return res;
}

void MissionPlannerWidget::gpsCb(const tobas_msgs::Gps::ConstSharedPtr& gps)
{
  gps_ = gps;
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
  if (res != QDialog::Accepted)
    return;

  const auto cmd_type = dialog.selectedCommand();
  BaseCommandWidget* cmd_widget;
  switch (cmd_type)
  {
    case command_t::WAYPOINT:
    {
      const auto [latitude, longitude] = map_->getCenter();
      const auto waypoint = new WaypointWidget();
      waypoint->latitude(latitude);
      waypoint->longitude(longitude);
      cmd_widget = waypoint;
      break;
    }
    case command_t::TAKEOFF:
    {
      cmd_widget = new TakeoffWidget();
      break;
    }
    case command_t::LAND:
    {
      cmd_widget = new LandWidget();
      break;
    }
    case command_t::RETURN_TO_HOME:
    {
      cmd_widget = new ReturnToHomeWidget();
      break;
    }
    default:
    {
      throw;
    }
  }

  const auto item = new QListWidgetItem(commandToText(cmd_type));
  command_list_->addItem(item);

  commands_->addWidget(cmd_widget);
  connect(cmd_widget, &BaseCommandWidget::updated, this, &self::onMissionUpdated);
  connect(
    cmd_widget, &BaseCommandWidget::deleteButtonClicked,
    std::bind(&self::onDeleteButtonClicked, this, item, cmd_widget));

  pairs_.insert({ item, cmd_widget });

  listToCommands();
  commandsToMap();
}

void MissionPlannerWidget::onClearButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onClearButtonClicked");

  if (!qt::yesOrNo(this, "Do you want to clear all the commands?", qt::QMessageLevel::WARN))
    return;

  map_->clear();
  command_list_->clear();
  commands_->clear();
  pairs_.clear();
}

void MissionPlannerWidget::onCacheButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onCacheButtonClicked");

  if (!qt::yesOrNo(this, "Do you want to cache map tiles to offline storage?", qt::QMessageLevel::WARN))
    return;

  const auto dir_from = linux::expandUser(kCacheDirOnline);
  const auto dir_to = linux::expandUser(kCacheDirOffline);

  if (!fs::is_directory(dir_to))
    fs::create_directories(dir_to);

  // 全てのPNGファイルをコピー
  for (const auto& entry : fs::directory_iterator(dir_from))
  {
    if (entry.path().extension() == ".png")
    {
      const auto& file_from = entry.path();
      const auto file_to = dir_to / file_from.filename();
      fs::copy_file(file_from, file_to, fs::copy_options::overwrite_existing);
    }
  }

  // ディレクトリ内のPNGファイルを取得
  std::vector<fs::path> files;
  for (const auto& entry : fs::directory_iterator(dir_to))
    if (entry.path().extension() == ".png")
      files.push_back(entry.path());

  // PNGファイルを最終変更時刻が新しい順にソート
  std::sort(
    files.begin(), files.end(),
    [](const fs::path& a, const fs::path& b) { return fs::last_write_time(a) > fs::last_write_time(b); });

  // ファイルサイズを取得
  std::vector<uintmax_t> sizes;
  for (const auto& file : files)
    sizes.push_back(fs::file_size(file));

  // ファイルサイズの累積和を計算
  std::vector<uintmax_t> sizes_cs(sizes.size());
  std::partial_sum(sizes.begin(), sizes.end(), sizes_cs.begin());

  // 最大サイズを超える最初の位置を見つける
  auto it = std::lower_bound(sizes_cs.begin(), sizes_cs.end(), kCacheMaxSize);
  const auto last_alive_idx = std::distance(sizes_cs.begin(), it);

  // サイズがリミットを超えたファイルを削除
  for (size_t i = last_alive_idx; i < files.size(); ++i)
    if (!fs::remove(files[i]))
      RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to remove " << files[i]);

  qt::qInfoBox(this, std::format("Map tiles are cached to {}.", kCacheDirOffline).c_str());
}

void MissionPlannerWidget::onExecuteButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onExecuteButtonClicked");

  if (!qt::yesOrNo(this, "Do you want to execute the mission?", qt::QMessageLevel::WARN))
    return;

  // ミッションが設定されているかどうかを確認
  if (command_list_->count() == 0)
  {
    qt::qWarnBox(this, "Mission is empty.");
    return;
  }

  // ミッションデータを抽出
  const auto mission_commands = createMissionCommandList();

  // TODO: 有効なミッションかどうかを確認 (Takeoff後にTakeoffはダメとか)

  // 実行モードに切り替える
  setExecuteMode();

  // ユーザ操作をブロックしないように別スレッドでミッションを実行
  mission_thread_.setNamespace(ns_);
  mission_thread_.setCommands(mission_commands);
  mission_thread_.start();
}

void MissionPlannerWidget::onCancelButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onCancelButtonClicked");

  if (!qt::yesOrNo(this, "Do you want to cancel the mission?", qt::QMessageLevel::WARN))
    return;

  // ミッションを停止
  mission_thread_.stop();

  // 編集モードに切り替える
  setEditMode();

  qt::qInfoBox(this, "The mission is canceled.");
}

void MissionPlannerWidget::onFocusButtonClicked()
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onFocusButtonClicked");

  if (gps_ == nullptr)
  {
    qt::qWarnBox(this, "GNSS data is not received yet.");
    return;
  }

  map_->setCenter(gps_->latitude, gps_->longitude);
}

void MissionPlannerWidget::onDeleteButtonClicked(QListWidgetItem* target_item, BaseCommandWidget* target_widget)
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onDeleteButtonClicked");

  command_list_->remove(target_item);
  commands_->removeWidget(target_widget);

  bool found = false;
  for (const auto& pair : pairs_)
  {
    const auto& item = pair.first;
    const auto& widget = pair.second;
    if (item == target_item && widget == target_widget)
    {
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

  if (mission_thread_.isRunning())
  {
    qt::qWarnBox(this, "You cannot edit the mission while executing it.");
    commandsToMap();
    return;
  }

  int cur_idx = 0;
  for (int i = 0; i < command_list_->count(); ++i)
  {
    const auto item = command_list_->item(i);
    const auto cmd_type = textToCommand(item->text().toUtf8());
    if (cmd_type == command_t::WAYPOINT)
      ++cur_idx;
    if (cur_idx == index)
    {
      const auto waypoint = qobject_cast<WaypointWidget*>(getCommandWidget(item));
      waypoint->latitude(latitude);
      waypoint->longitude(longitude);
      break;
    }
  }

  if (cur_idx != index)
    throw std::runtime_error(std::format("Index {} is out of range.", index));
}

void MissionPlannerWidget::onMissionFinished(bool success, const QString& message)
{
  RCLCPP_DEBUG(node_->get_logger(), "MissionPlannerWidget::onMissionFinished");

  if (success)
    qt::qInfoBox(this, "The mission is completed.");
  else
    qt::qErrorBox(this, message);

  setEditMode();
}
}  // namespace control_system
}  // namespace gui
