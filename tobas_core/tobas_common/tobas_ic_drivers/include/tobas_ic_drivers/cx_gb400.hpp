#pragma once

#include <tobas_linux/video_dev.hpp>

#include <chrono>

#define PACKED __attribute__((__packed__))  // 構造体のメンバ変数がメモリ上で連続する

namespace driver
{
/**
 * @brief A linux driver of cx_gb100 camera.
 */
class CxGb400 : public linux::VideoDev
{
public:
  static constexpr std::chrono::milliseconds att_send_interval = std::chrono::milliseconds(200);
  enum cam_pos_t : uint8_t
  {
    LOWER = 0x0,           // 機体に下向きで取り付ける（レンズより台が上になる向き）
    UPPER = 0x1,           // 機体に上向きで取り付ける（レンズより台が下になる向き）
    UPPER_YAW_FIX = 0x02,  // 機体に上向きで取り付ける　ヨーは回転フリー
    AUTO = 0x03            // 自動で決定　デフォルト値
  };

  explicit CxGb400();
  ~CxGb400();

  bool initialize(const char* video_dev, const cam_pos_t& camera_position = AUTO, bool disable_fullHD = true, bool disable_video_streaming = false);
  bool sendCopterAttitude(const double& q_w, const double& q_x, const double& q_y, const double& q_z);
  // Send gimbal attitude control command. Specify angles in degrees satisfing -115.0 < pitch_deg < 45.0, -85.0 < yaw_deg < 85.0.
  bool sendGimbalCtrl(const double& pitch_deg, const double& yaw_deg);
  // 工場出荷リセット．
  bool fullReset();
  // 工場出荷リセット実施後，通常起動前に1回のみ実施する
  bool turnOffUAVCAN();

private:
  static constexpr double kPitchCmdMax = 45.0;
  static constexpr double kPitchCmdMin = -115.0;
  static constexpr double kYawCmdMax = 85.0;
  static constexpr double kYawCmdMin = -85.0;
  static constexpr double kGimbalAngleResolution = 1.0 / 0.01;

  enum unit_id_t : uint8_t
  {
    UNIT1 = 0x6,
    UNIT2 = 0x7,
    UNIT3 = 0x8
  };

  struct PACKED AttitudeMsg
  {
    int16_t q_w;
    int16_t q_x;
    int16_t q_y;
    int16_t q_z;
  };

  struct PACKED GimbalCtrlMsg
  {
    uint8_t pitch_cmd_type;
    uint8_t yaw_cmd_type;
    int16_t pitch_cmd_value;
    int16_t yaw_cmd_value;
  };
};

}  // namespace driver
