#pragma once

#include <tobas_kdl/rotation.hpp>
#include <tobas_msgs_adapter/pos_vel_acc_yaw.hpp>
#include <tobas_msgs_adapter/pose_twist_accel_command.hpp>

namespace tobas
{
kdl::Rotation rotWorldToFootprint(const kdl::Rotation& R_W_B);

kdl::Rotation rotFootprintToLocal(const kdl::Rotation& R_W_B);

/**
 * @brief コマンドメッセージのフレームを変換する．
 *
 * @param frame_id 変換先のフレーム
 * @param R_W_B 世界座標系に対する機体座標系の姿勢行列
 * @param msg コマンドメッセージ
 * @return true 変換に成功した場合
 * @return false 変換に失敗した場合
 */
bool changeFrame(const uint8_t& frame_id, const kdl::Rotation& R_W_B, tobas_msgs::PosVelAccYaw& msg);

/**
 * @brief コマンドメッセージのフレームを変換する．
 *
 * @param frame_id 変換先のフレーム
 * @param R_W_B 世界座標系に対する機体座標系の姿勢行列
 * @param msg コマンドメッセージ
 * @return true 変換に成功した場合
 * @return false 変換に失敗した場合
 */
bool changeFrame(const uint8_t& frame_id, const kdl::Rotation& R_W_B, tobas_msgs::PoseTwistAccelCommand& msg);
}  // namespace tobas
