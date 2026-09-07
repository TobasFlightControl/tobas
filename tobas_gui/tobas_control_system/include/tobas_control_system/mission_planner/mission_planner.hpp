// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QSettings>
#include <rclcpp_action/client.hpp>

#include <tobas_geographic/geography.hpp>
#include <tobas_mission_items/mission.hpp>
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

Q_SIGNALS:
  void goalResponseReceived(bool ok);
  void feedbackReceived(uint32_t current_command_index);
  void resultReceived(rclcpp_action::ResultCode code, const QString& message, uint32_t last_cmd_idx);

public:
  explicit MissionPlannerWidget(const rqt::RosQtBridge& bridge);

  void reset();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);
  void clearRosInterfaces();

private:
  QSettings settings_store_;
  geo::Geography geography_;

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

  bool ros_initialized_ = false;
  bool mission_executing_ = false;
  Client::SharedPtr mission_ac_;

  QString getMissionDir();
  void setMissionDir(const QString& file_path);

  void clearMission();

  /* Add a mission command to the planner. */
  void addCommand(mission::Type type, BaseCommandWidget* widget);

  /* Switch each widget to execute mode. */
  void setExecuteMode();

  /* Switch each widget to edit mode. */
  void setEditMode();

  /* Update the displayed command widget based on the selected list item. */
  void listToCommands();

  /* Redraw map objects based on the current commands. */
  void commandsToMap();

  /* Update whether each `Waypoint` is a spline endpoint from the command sequence. */
  void updateWaypointSplineEnds();

  /* Draw the `Waypoint` sequence on the map as a spline. */
  void addSplinePathToMap(const std::vector<QGeoCoordinate>& waypoints);

  /* Get the command widget corresponding to the list item. */
  BaseCommandWidget* findCommandWidget(const QListWidgetItem* _item);
  const BaseCommandWidget* findCommandWidget(const QListWidgetItem* _item) const;

  /* Get the last waypoint widget. */
  const WaypointWidget* findLastWaypoint() const;

  /* Create a mission from the current settings. */
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
