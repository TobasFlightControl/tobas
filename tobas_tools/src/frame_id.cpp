#include "../include/tobas_tools/conversions/frame_id.hpp"

using namespace KDL;
using namespace tobas_msgs;

namespace tobas
{
bool changeFrame(const uint8_t& frame_id, const Rotation& R_gl, PosVelAccYaw& msg)
{
  switch (frame_id)
  {
    // 変換先がグローバル座標系の場合
    case FrameId::GLOBAL:
    {
      // Velocity
      switch (msg.vel_frame.data)
      {
        case FrameId::GLOBAL:
          break;
        case FrameId::LOCAL:
          msg.vel = R_gl * msg.vel;
          break;
        default:
          return false;
      }

      // Acceleration
      switch (msg.acc_frame.data)
      {
        case FrameId::GLOBAL:
          break;
        case FrameId::LOCAL:
          msg.acc = R_gl * msg.acc;
          break;
        default:
          return false;
      }

      break;
    }

    // 変換先がローカル座標系の場合
    case FrameId::LOCAL:
    {
      // Velocity
      switch (msg.vel_frame.data)
      {
        case FrameId::GLOBAL:
          msg.vel = R_gl.inverse(msg.vel);
          break;
        case FrameId::LOCAL:
          break;
        default:
          return false;
      }

      // Acceleration
      switch (msg.acc_frame.data)
      {
        case FrameId::GLOBAL:
          msg.acc = R_gl.inverse(msg.acc);
          break;
        case FrameId::LOCAL:
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
  msg.vel_frame.data = frame_id;
  msg.acc_frame.data = frame_id;

  return true;
}
}  // namespace tobas
