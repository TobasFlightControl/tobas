#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_msgs_adapter/gnss.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "./map_widget.hpp"
#include "./command_button.hpp"
#include "./mission_execution_thread.hpp"
#include "./add_command_dialog.hpp"

namespace gui
{
namespace gcs
{
class MissionPlannerWidget : public QWidget
{
  Q_OBJECT

  using self = MissionPlannerWidget;
  using super = QWidget;

  static constexpr char kCacheDirOnline[] = "~/.cache/tobas/tiles/online/";
  static constexpr char kCacheDirOffline[] = "~/.cache/tobas/tiles/offline/";
  static constexpr uintmax_t kCacheMaxSize = 1 << 30;  // 1GiB

public:
  explicit MissionPlannerWidget(rclcpp::Node::SharedPtr node);

  void reset();
  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

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
  MissionExecutionThread mission_thread_;

  ros2::SubscriberPtr<tobas_msgs::Gnss> gnss_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;

  /* 各ウィジェットを実行モードに切り替える． */
  void setExecuteMode();

  /* 各ウィジェットを編集モードに切り替える． */
  void setEditMode();

  /* 選択されているリストアイテムに基づいてコマンドウィジェットの表示を更新． */
  void listToCommands();

  /* 現在のコマンドに基づいてマップ上のオブジェクトを描き直す． */
  void commandsToMap();

  /* リストの要素に対応するコマンドウィジェットを取得する． */
  BaseCommandWidget* getCommandWidget(QListWidgetItem* tar_item);

  /* ミッションコマンドのリストを作成する． */
  QVector<BaseCommandData::SharedPtr> createMissionCommandList();

  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);

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
  void onMissionFinished(bool success, const QString& message);
};
}  // namespace gcs
}  // namespace gui
