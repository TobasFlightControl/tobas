// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <vector>

#include <rclcpp_action/rclcpp_action.hpp>

#include <tobas_mission_items/mission.hpp>
#include <tobas_property_client/property_client.hpp>
#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include <tobas_mission_msgs/action/execute_mission.hpp>

#include "./add_command_dialog.hpp"
#include "./command_button.hpp"
#include "./commands/commands.hpp"
#include "./map/map.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
class MissionPlannerWidget : public QWidget
{
  Q_OBJECT

  using self = MissionPlannerWidget;
  using super = QWidget;

  using Action = tobas_mission_msgs::action::ExecuteMission;
  using Client = rclcpp_action::Client<Action>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  static constexpr char kCacheDirOnline[] = "~/.cache/tobas/tiles/online";
  static constexpr char kCacheDirOffline[] = "~/.cache/tobas/tiles/offline";
  static constexpr uintmax_t kCacheMaxSize = 1 << 30;  // 1GiB

  static constexpr char kLastOpenedDirKey[] = "last_opened_dir";

Q_SIGNALS:
  void goalResponseReceived(bool ok);
  void feedbackReceived(uint32_t current_command_index);
  void resultReceived(rclcpp_action::ResultCode code, const QString& message, uint32_t last_cmd_idx);

public:
  explicit MissionPlannerWidget(rclcpp::Node::SharedPtr node, const RosQtBridge& bridge);

  void reset();
  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;
  ptree::PropertyClient property_client_;

  MapWidget* map_;

  CommandButton* load_button_;
  CommandButton* save_button_;
  CommandButton* add_button_;
  CommandButton* clear_button_;
  CommandButton* cache_button_;
  CommandButton* execute_button_;
  CommandButton* cancel_button_;
  CommandButton* focus_button_;

  qt::ListWidget* command_list_;
  qt::StackedWidget* commands_;
  std::set<std::pair<QListWidgetItem*, BaseCommandWidget*>> pairs_;

  qt::WaitSpinnerWidget spinner_;

  bool mission_executing_;
  Client::SharedPtr mission_ac_;

  QString getMissionDir();
  void setMissionDir(const QString& file_path);

  void clearMission();

  /* プランナーにミッションコマンドを追加する． */
  void addCommand(mission::Type type, BaseCommandWidget* widget);

  /* 各ウィジェットを実行モードに切り替える． */
  void setExecuteMode();

  /* 各ウィジェットを編集モードに切り替える． */
  void setEditMode();

  /* 選択されているリストアイテムに基づいてコマンドウィジェットの表示を更新する． */
  void listToCommands();

  /* 現在のコマンドに基づいてマップ上のオブジェクトを描き直す． */
  void commandsToMap();

  /* コマンド列から各Waypointがスプライン終端かどうかを更新する． */
  void updateWaypointSplineEnds();

  /* Waypoint列をスプラインとしてマップに描画する． */
  void addSplinePathToMap(const std::vector<QGeoCoordinate>& waypoints);

  /* リストの要素に対応するコマンドウィジェットを取得する． */
  BaseCommandWidget* findCommandWidget(const QListWidgetItem* _item);
  const BaseCommandWidget* findCommandWidget(const QListWidgetItem* _item) const;

  /* 最後のウェイポイントウィジェットを取得する． */
  const WaypointWidget* findLastWaypoint() const;

  /* 現在の設定からミッションを作成する． */
  tobas::mission::Mission createMission() const;

private Q_SLOTS:
  void onLoadButtonClicked();
  void onSaveButtonClicked();
  void onAddButtonClicked();
  void onClearButtonClicked();
  void onCacheButtonClicked();
  void onExecuteButtonClicked();
  void onCancelButtonClicked();
  void onFocusButtonClicked();

  void onDeleteButtonClicked(QListWidgetItem* target_item, BaseCommandWidget* target_widget);
  void onListItemChanged();
  void onMissionUpdated();
  void onWaypointMoved(int index, double latitude, double longitude);

  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss);
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom);

  void actionGoalResponseCb(bool ok);
  void actionFeedbackCb(uint32_t cur_cmd_idx);
  void actionResultCb(rclcpp_action::ResultCode code, const QString& message, uint32_t last_cmd_idx);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
