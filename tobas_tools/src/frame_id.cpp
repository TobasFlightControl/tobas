#include "../include/tobas_tools/conversions/frame_id.hpp"

namespace tobas
{
kdl::Rotation rotWorldToFootprint(const kdl::Rotation& R_W_B)
{
  return kdl::Rotation::RotZ(R_W_B.getYaw());
}

kdl::Rotation rotFootprintToLocal(const kdl::Rotation& R_W_B)
{
  double roll, pitch, yaw;
  R_W_B.getRPY(roll, pitch, yaw);
  return kdl::Rotation::RPY(roll, pitch, 0);
}

bool changeFrame(const uint8_t& frame_id, const kdl::Rotation& R_W_B, tobas_command_msgs::PosVelAccYaw& msg)
{
  switch (frame_id)
  {
    // 変換先がグローバル座標系の場合
    case tobas_command_msgs::msg::FrameId::WORLD:
    {
      switch (msg.frame_id.data)
      {
        case tobas_command_msgs::msg::FrameId::WORLD:
          break;
        case tobas_command_msgs::msg::FrameId::LOCAL:
          msg.vel = R_W_B * msg.vel;
          msg.acc = R_W_B * msg.acc;
          break;
        case tobas_command_msgs::msg::FrameId::FOOTPRINT:
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
    case tobas_command_msgs::msg::FrameId::LOCAL:
    {
      switch (msg.frame_id.data)
      {
        case tobas_command_msgs::msg::FrameId::WORLD:
          msg.vel = R_W_B.inverse(msg.vel);
          msg.acc = R_W_B.inverse(msg.acc);
          break;
        case tobas_command_msgs::msg::FrameId::LOCAL:
          break;
        case tobas_command_msgs::msg::FrameId::FOOTPRINT:
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
    case tobas_command_msgs::msg::FrameId::FOOTPRINT:
    {
      switch (msg.frame_id.data)
      {
        case tobas_command_msgs::msg::FrameId::WORLD:
        {
          const auto R_W_F = rotWorldToFootprint(R_W_B);
          msg.vel = R_W_F.inverse(msg.vel);
          msg.acc = R_W_F.inverse(msg.acc);
          break;
        }
        case tobas_command_msgs::msg::FrameId::LOCAL:
        {
          const auto R_F_L = rotFootprintToLocal(R_W_B);
          msg.vel = R_F_L * msg.vel;
          msg.acc = R_F_L * msg.acc;
          break;
        }
        case tobas_command_msgs::msg::FrameId::FOOTPRINT:
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

bool changeFrame(const uint8_t& frame_id, const kdl::Rotation& R_W_B, tobas_command_msgs::PoseTwistAccelCommand& msg)
{
  switch (frame_id)
  {
    // 変換先がグローバル座標系の場合
    case tobas_command_msgs::msg::FrameId::WORLD:
    {
      switch (msg.frame_id.data)
      {
        case tobas_command_msgs::msg::FrameId::WORLD:
          break;
        case tobas_command_msgs::msg::FrameId::LOCAL:
          msg.vel = R_W_B * msg.vel;
          msg.acc = R_W_B * msg.acc;
          msg.gyro = R_W_B * msg.gyro;
          msg.dgyro = R_W_B * msg.dgyro;
          break;
        case tobas_command_msgs::msg::FrameId::FOOTPRINT:
        {
          const auto R_W_F = rotWorldToFootprint(R_W_B);
          msg.vel = R_W_F * msg.vel;
          msg.acc = R_W_F * msg.acc;
          msg.gyro = R_W_F * msg.gyro;
          msg.dgyro = R_W_F * msg.dgyro;
          break;
        }
        default:
          return false;
      }

      break;
    }

    // 変換先がローカル座標系の場合
    case tobas_command_msgs::msg::FrameId::LOCAL:
    {
      switch (msg.frame_id.data)
      {
        case tobas_command_msgs::msg::FrameId::WORLD:
          msg.vel = R_W_B.inverse(msg.vel);
          msg.acc = R_W_B.inverse(msg.acc);
          msg.gyro = R_W_B.inverse(msg.gyro);
          msg.dgyro = R_W_B.inverse(msg.dgyro);
          break;
        case tobas_command_msgs::msg::FrameId::LOCAL:
          break;
        case tobas_command_msgs::msg::FrameId::FOOTPRINT:
        {
          const auto R_F_L = rotFootprintToLocal(R_W_B);
          msg.vel = R_F_L.inverse(msg.vel);
          msg.acc = R_F_L.inverse(msg.acc);
          msg.gyro = R_F_L.inverse(msg.gyro);
          msg.dgyro = R_F_L.inverse(msg.dgyro);
          break;
        }
        default:
          return false;
      }

      break;
    }

    // 変換先がフットプリントの場合
    case tobas_command_msgs::msg::FrameId::FOOTPRINT:
    {
      switch (msg.frame_id.data)
      {
        case tobas_command_msgs::msg::FrameId::WORLD:
        {
          const auto R_W_F = rotWorldToFootprint(R_W_B);
          msg.vel = R_W_F.inverse(msg.vel);
          msg.acc = R_W_F.inverse(msg.acc);
          msg.gyro = R_W_F.inverse(msg.gyro);
          msg.dgyro = R_W_F.inverse(msg.dgyro);
          break;
        }
        case tobas_command_msgs::msg::FrameId::LOCAL:
        {
          const auto R_F_L = rotFootprintToLocal(R_W_B);
          msg.vel = R_F_L * msg.vel;
          msg.acc = R_F_L * msg.acc;
          msg.gyro = R_F_L * msg.gyro;
          msg.dgyro = R_F_L * msg.dgyro;
          break;
        }
        case tobas_command_msgs::msg::FrameId::FOOTPRINT:
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
