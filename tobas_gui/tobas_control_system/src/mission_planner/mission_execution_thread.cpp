#include "tobas_control_system/mission_planner/mission_execution_thread.hpp"

#include <boost/polymorphic_pointer_cast.hpp>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>

using namespace std;
using namespace tobas_msgs::srv;
using namespace tobas_mission_msgs::action;

namespace gui
{
namespace gcs
{
MissionExecutionThread::MissionExecutionThread(rclcpp::Node::SharedPtr node) : node_(node)
{
}

void MissionExecutionThread::run()
{
  // Reset
  stop_requested_ = false;

  // Create service clients
  get_gnss_origin_sc_ = make_shared<ros2::SyncServiceClient<GetGnssOrigin>>(
    node_, path::join(ns_, tobas::kRemoteIfaceTopicNS, tobas::kGetGnssOriginSrv));

  // Create action clients
  takeoff_ac_ = make_shared<ros2::SyncActionClient<Takeoff>>(node_, path::join(ns_, tobas::kTakeoffAction));
  land_ac_ = make_shared<ros2::SyncActionClient<Land>>(node_, path::join(ns_, tobas::kLandAction));
  move_ac_ = make_shared<ros2::SyncActionClient<Move>>(node_, path::join(ns_, tobas::kMoveAction));

  // 前から順にコマンドを実行
  for (const auto& command : commands_) {
    if (!execute(command)) {
      return;
    }
  }

  Q_EMIT finished(true, "");
}

void MissionExecutionThread::setNamespace(const string& ns)
{
  ns_ = ns;
}

void MissionExecutionThread::setCommands(const QVector<BaseCommandData::SharedPtr>& commands)
{
  commands_ = commands;
}

void MissionExecutionThread::stop()
{
  stop_requested_ = true;
}

bool MissionExecutionThread::execute(BaseCommandData::SharedPtr command)
{
  const auto cmd_type = command->type();
  RCLCPP_INFO_STREAM(node_->get_logger(), "Execute command: " << commandToText(cmd_type));

  switch (cmd_type) {
    case command_t::WAYPOINT:
      return execute(boost::polymorphic_pointer_downcast<WaypointData>(command));
    case command_t::TAKEOFF:
      return execute(boost::polymorphic_pointer_downcast<TakeoffData>(command));
    case command_t::LAND:
      return execute(boost::polymorphic_pointer_downcast<LandData>(command));
    case command_t::RETURN_TO_HOME:
      return execute(boost::polymorphic_pointer_downcast<ReturnToHomeData>(command));
    default:
      Q_EMIT finished(false, format("Unknown command type: {}", (int)cmd_type).c_str());
      return false;
  }
}

bool MissionExecutionThread::execute(TakeoffData::SharedPtr command)
{
  // ゴールを作成
  Takeoff::Goal goal;
  goal.level.data = kCommandLevel;
  goal.target_altitude = command->altitude;
  goal.altitude_tolerance = command->altitude_tolerance;
  goal.duration = command->duration;
  goal.timeout = kCommandTimeout;

  // アクションを実行
  auto [goal_handle, get_result_future] = takeoff_ac_->sendGoal(goal);
  if (!get_result_future.valid()) {
    Q_EMIT finished(false, "Failed to call takeoff action.");
    return false;
  }

  // 終了フラグを監視しながら待機
  while (get_result_future.wait_for(kCheckCancelInterval) != std::future_status::ready) {
    if (stop_requested_) {
      takeoff_ac_->cancelGoal(goal_handle);
      return false;
    }
  }

  // アクションの成否を確認
  const auto result = get_result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    Q_EMIT finished(false, result.result->message.c_str());
    return false;
  }

  return true;
}

bool MissionExecutionThread::execute(LandData::SharedPtr)
{
  // ゴールを作成
  Land::Goal goal;
  goal.level.data = kCommandLevel;

  // アクションを実行
  auto [goal_handle, get_result_future] = land_ac_->sendGoal(goal);
  if (!get_result_future.valid()) {
    Q_EMIT finished(false, "Failed to call land action.");
    return false;
  }

  // 終了フラグを監視しながら待機
  while (get_result_future.wait_for(kCheckCancelInterval) != std::future_status::ready) {
    if (stop_requested_) {
      land_ac_->cancelGoal(goal_handle);
      return false;
    }
  }

  // アクションの成否を確認
  const auto result = get_result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    Q_EMIT finished(false, result.result->message.c_str());
    return false;
  }

  return true;
}

bool MissionExecutionThread::execute(WaypointData::SharedPtr command)
{
  // ゴールを作成
  Move::Goal goal;
  goal.level.data = kCommandLevel;
  goal.target_latitude = command->latitude;
  goal.target_longitude = command->longitude;
  goal.target_altitude = command->altitude;
  goal.acceptance_radius = command->acceptance_radius;
  goal.duration = command->duration;
  goal.timeout = kCommandTimeout;

  // アクションを実行
  auto [goal_handle, get_result_future] = move_ac_->sendGoal(goal);
  if (!get_result_future.valid()) {
    Q_EMIT finished(false, "Failed to call move action.");
    return false;
  }

  // 終了フラグを監視しながら待機
  while (get_result_future.wait_for(kCheckCancelInterval) != std::future_status::ready) {
    if (stop_requested_) {
      move_ac_->cancelGoal(goal_handle);
      return false;
    }
  }

  // アクションの成否を確認
  const auto result = get_result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    Q_EMIT finished(false, result.result->message.c_str());
    return false;
  }

  return true;
}

bool MissionExecutionThread::execute(ReturnToHomeData::SharedPtr command)
{
  // ホームポジションの経緯度を取得
  const auto get_gnss_origin_req = make_shared<tobas_msgs::srv::GetGnssOrigin::Request>();
  if (!get_gnss_origin_sc_->call(get_gnss_origin_req)) {
    Q_EMIT finished(false, format("Failed to call \"{}\" service.", tobas::kGetGnssOriginSrv).c_str());
    return false;
  }

  const auto get_gnss_origin_res = get_gnss_origin_sc_->getResponse();
  if (!get_gnss_origin_res->success) {
    Q_EMIT finished(false, format("Failed to get GNSS origin: {}", get_gnss_origin_res->message).c_str());
    return false;
  }

  // ゴールを作成
  Move::Goal goal;
  goal.level.data = kCommandLevel;
  goal.target_latitude = get_gnss_origin_res->latitude;
  goal.target_longitude = get_gnss_origin_res->longitude;
  goal.target_altitude = command->altitude;
  goal.acceptance_radius = command->acceptance_radius;
  goal.duration = command->duration;
  goal.timeout = kCommandTimeout;

  // アクションを実行
  auto [goal_handle, get_result_future] = move_ac_->sendGoal(goal);
  if (!get_result_future.valid()) {
    Q_EMIT finished(false, "Failed to call move action.");
    return false;
  }

  // 終了フラグを監視しながら待機
  while (get_result_future.wait_for(kCheckCancelInterval) != std::future_status::ready) {
    if (stop_requested_) {
      move_ac_->cancelGoal(goal_handle);
      return false;
    }
  }

  // アクションの成否を確認
  const auto result = get_result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    Q_EMIT finished(false, result.result->message.c_str());
    return false;
  }

  return true;
}
}  // namespace gcs
}  // namespace gui
