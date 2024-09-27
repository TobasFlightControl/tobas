#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_msgs_adapter/Gps.hpp>
#include <tobas_qt_tools/widgets/list_widget.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "./map_widget.hpp"
#include "./mission_execution_thread.hpp"
#include "./add_command_dialog.hpp"

namespace gui
{
namespace control_system
{
class MissionPlannerWidget : public QWidget
{
  Q_OBJECT

  using self = MissionPlannerWidget;
  using super = QWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

  static constexpr char kCacheDirOnline[] = "~/.cache/tobas/tiles/online/";
  static constexpr char kCacheDirOffline[] = "~/.cache/tobas/tiles/offline/";
  static constexpr uintmax_t kCacheMaxSize = 1 << 30;  // 1GiB

public:
  explicit MissionPlannerWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  std::string ns_;

  MapWidget* map_;

  QPushButton* load_button_;
  QPushButton* save_button_;
  QPushButton* add_button_;
  QPushButton* clear_button_;
  QPushButton* cache_button_;
  QPushButton* execute_button_;
  QPushButton* cancel_button_;
  QPushButton* focus_button_;

  qt::ListWidget* command_list_;
  qt::StackedWidget* commands_;

  std::set<std::pair<QListWidgetItem*, BaseCommandWidget*>> pairs_;
  MissionExecutionThread mission_thread_;

  tobas_msgs::Gps::ConstSharedPtr gps_;

  ros2::SubscriberPtr<tobas_msgs::Gps> gps_sub_;

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

  void gpsCb(const tobas_msgs::Gps::ConstSharedPtr& gps);

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
}  // namespace control_system
}  // namespace gui
