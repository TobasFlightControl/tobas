#pragma once

#include <QThread>

#include <tobas_ros2_tools/sync_action_client.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

#include <tobas_mission_msgs/action/land.hpp>
#include <tobas_mission_msgs/action/move.hpp>
#include <tobas_mission_msgs/action/takeoff.hpp>
#include <tobas_msgs/srv/get_gnss_origin.hpp>

#include "./commands/commands.hpp"

namespace gui
{
namespace gcs
{
class MissionExecutionThread : public QThread
{
  Q_OBJECT

  using self = MissionExecutionThread;
  using super = QThread;

  static constexpr auto kCommandLevel = tobas_command_msgs::msg::CommandLevel::NORMAL;
  static constexpr auto kCommandTimeout = 10.;  // TODO: ユーザが設定できるようにする
  static constexpr auto kCheckCancelInterval = std::chrono::milliseconds(100);

Q_SIGNALS:
  // 実行結果を返すためのシグナル．
  // NOTE: QThreadでGUIを使うとメインスレッドを壊す恐れがあるため，シグナルスロット以外は使用しない．
  void finished(bool success, const QString& message);

public:
  explicit MissionExecutionThread(rclcpp::Node::SharedPtr node);

  void run() override;

  void setNamespace(const std::string& ns);
  void setCommands(const QVector<BaseCommandData::SharedPtr>& commands);

  /**
   * @brief 実行中の関数に割り込んでミッションの終了フラグを立てる．
   * @note QThread.terminate()はクラッシュの恐れがあるため呼ばない．
   */
  void stop();

private:
  const rclcpp::Node::SharedPtr node_;

  std::string ns_;
  QVector<BaseCommandData::SharedPtr> commands_;

  bool stop_requested_;

  ros2::SyncServiceClient<tobas_msgs::srv::GetGnssOrigin>::SharedPtr get_gnss_origin_sc_;
  ros2::SyncActionClient<tobas_mission_msgs::action::Takeoff>::SharedPtr takeoff_ac_;
  ros2::SyncActionClient<tobas_mission_msgs::action::Land>::SharedPtr land_ac_;
  ros2::SyncActionClient<tobas_mission_msgs::action::Move>::SharedPtr move_ac_;

  bool execute(BaseCommandData::SharedPtr command);

  bool execute(TakeoffData::SharedPtr command);
  bool execute(LandData::SharedPtr command);
  bool execute(WaypointData::SharedPtr command);
  bool execute(ReturnToHomeData::SharedPtr command);
};
}  // namespace gcs
}  // namespace gui
