#pragma once

#include <tobas_kdl/rotation.hpp>
#include <tobas_msgs/PosVelAccYaw.hpp>
#include <tobas_msgs/PoseTwistAccelCommand.hpp>

namespace tobas
{
kdl::Rotation rotWorldToFootprint(const kdl::Rotation& R_W_B);

kdl::Rotation rotFootprintToLocal(const kdl::Rotation& R_W_B);

/**
 * @brief コマンドメッセージのvel, accのフレームを変換する．
 *
 * @tparam T コマンドメッセージの型
 * @param frame_id 変換先のフレーム
 * @param R_W_B 世界座標系に対する機体座標系の姿勢行列
 * @param msg コマンドメッセージ
 * @return true 変換に成功した場合
 * @return false 変換に失敗した場合
 */
template <typename T>
bool changeFrame(const uint8_t& frame_id, const kdl::Rotation& R_W_B, T& msg)
{
  switch (frame_id)
  {
    // 変換先がグローバル座標系の場合
    case tobas_msgs::FrameId::WORLD:
    {
      switch (msg.frame_id.data)
      {
        case tobas_msgs::FrameId::WORLD:
          break;
        case tobas_msgs::FrameId::LOCAL:
          msg.vel = R_W_B * msg.vel;
          msg.acc = R_W_B * msg.acc;
          break;
        case tobas_msgs::FrameId::FOOTPRINT:
        {
          const auto R_W_F = rotWorldToFootprint(R_W_B);
          msg.vel = R_W_F * msg.vel;
          msg.acc = R_W_F * msg.acc;
          break;
        }
        default:
          return false;
      }

      break;
    }

    // 変換先がローカル座標系の場合
    case tobas_msgs::FrameId::LOCAL:
    {
      switch (msg.frame_id.data)
      {
        case tobas_msgs::FrameId::WORLD:
          msg.vel = R_W_B.inverse(msg.vel);
          msg.acc = R_W_B.inverse(msg.acc);
          break;
        case tobas_msgs::FrameId::LOCAL:
          break;
        case tobas_msgs::FrameId::FOOTPRINT:
        {
          const auto R_F_L = rotFootprintToLocal(R_W_B);
          msg.vel = R_F_L.inverse(msg.vel);
          msg.acc = R_F_L.inverse(msg.acc);
          break;
        }
        default:
          return false;
      }

      break;
    }

    // 変換先がフットプリントの場合
    case tobas_msgs::FrameId::FOOTPRINT:
    {
      switch (msg.frame_id.data)
      {
        case tobas_msgs::FrameId::WORLD:
        {
          const auto R_W_F = rotWorldToFootprint(R_W_B);
          msg.vel = R_W_F.inverse(msg.vel);
          msg.acc = R_W_F.inverse(msg.acc);
          break;
        }
        case tobas_msgs::FrameId::LOCAL:
        {
          const auto R_F_L = rotFootprintToLocal(R_W_B);
          msg.vel = R_F_L * msg.vel;
          msg.acc = R_F_L * msg.acc;
          break;
        }
        case tobas_msgs::FrameId::FOOTPRINT:
          break;
        default:
          return false;
      }

      break;
    }

    default:
    {
      return false;
    }
  }

  // FrameIdを更新
  msg.frame_id.data = frame_id;

  return true;
}
}  // namespace tobas
