#include "../include/tobas_tools/conversions/frame_id.hpp"

using namespace KDL;
using namespace tobas_msgs;

namespace tobas
{
Rotation rotWorldToFootprint(const Rotation& R_W_B)
{
  return Rotation::RotZ(R_W_B.getYaw());
}

Rotation rotFootprintToLocal(const Rotation& R_W_B)
{
  double roll, pitch, yaw;
  R_W_B.getRPY(roll, pitch, yaw);
  return Rotation::RPY(roll, pitch, 0);
}

bool changeFrame(const uint8_t& frame_id, const Rotation& R_W_B, PosVelAccYaw& msg)
{
  switch (frame_id)
  {
    // 変換先がグローバル座標系の場合
    case FrameId::WORLD:
    {
      switch (msg.frame_id.data)
      {
        case FrameId::WORLD:
          break;
        case FrameId::LOCAL:
          msg.vel = R_W_B * msg.vel;
          msg.acc = R_W_B * msg.acc;
          break;
        case FrameId::FOOTPRINT:
        {
          const auto R_W_F = rotWorldToFootprint(R_W_B);
          msg.vel = R_W_F * msg.vel;
          msg.acc = R_W_F * msg.acc;
        }
        break;
        default:
          return false;
      }

      break;
    }

    // 変換先がローカル座標系の場合
    case FrameId::LOCAL:
    {
      switch (msg.frame_id.data)
      {
        case FrameId::WORLD:
          msg.vel = R_W_B.inverse(msg.vel);
          msg.acc = R_W_B.inverse(msg.acc);
          break;
        case FrameId::LOCAL:
          break;
        case FrameId::FOOTPRINT:
        {
          const auto R_F_L = rotFootprintToLocal(R_W_B);
          msg.vel = R_F_L.inverse(msg.vel);
          msg.acc = R_F_L.inverse(msg.acc);
        }
        break;
        default:
          return false;
      }
    }

    // 変換先がフットプリントの場合
    case FrameId::FOOTPRINT:
    {
      switch (msg.frame_id.data)
      {
        case FrameId::WORLD:
        {
          const auto R_W_F = rotWorldToFootprint(R_W_B);
          msg.vel = R_W_F.inverse(msg.vel);
          msg.acc = R_W_F.inverse(msg.acc);
        }
        break;
        case FrameId::LOCAL:
        {
          const auto R_F_L = rotFootprintToLocal(R_W_B);
          msg.vel = R_F_L * msg.vel;
          msg.acc = R_F_L * msg.acc;
        }
        break;
        case FrameId::FOOTPRINT:
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
