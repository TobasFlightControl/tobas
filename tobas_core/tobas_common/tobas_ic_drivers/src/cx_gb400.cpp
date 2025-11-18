#include "tobas_ic_drivers/cx_gb400.hpp"

#include <iostream>

namespace driver
{
CxGb400::CxGb400() : linux::VideoDev::VideoDev()
{
}

CxGb400::~CxGb400()
{
}

bool CxGb400::initialize(
  const char* video_dev,
  const CameraPosition& camera_position,
  bool disable_fullHD,
  bool disable_video_streaming)
{
  if (!linux::VideoDev::initialize(video_dev, "MJPG", disable_video_streaming, 1280, 720)) {
    std::cerr << "Failed to initialize video device." << std::endl;
    return false;
  }
  // check UAVCAN is turned off (= UVC control is available)
  uint8_t is_uavcan_on = 0;
  struct uvc_xu_control_query check_UAVCAN_query = { kUnit2, 0x1E, UVC_GET_CUR, 1, &is_uavcan_on };
  if (!execUvcControl(check_UAVCAN_query)) {
    std::cerr << "Failed to check UAVCAN query." << std::endl;
    return false;
  }
  if (is_uavcan_on) {
    std::cerr << "UAVCAN is ON. Reset the camera and set UAVCAN OFF following the EXCEL file received from xacti."
              << std::endl;
    return false;
  }

  // set camera position
  uint8_t cam_pos_data = static_cast<uint8_t>(camera_position);
  struct uvc_xu_control_query set_cam_pos_query = { kUnit2, 0x1B, UVC_SET_CUR, 1, &cam_pos_data };
  if (!execUvcControl(set_cam_pos_query)) {
    std::cerr << "Failed to set camera position." << std::endl;
    return false;
  }

  // gymbal restart, どうやら必要ないっぽい　sendAttitudeを200ms間隔で送ると自動でrestartしてくれる
  // uint8_t execute = 0;
  // struct uvc_xu_control_query gymbal_restart = {kUnit2, 0xA, UVC_SET_CUR, 1, &execute};
  // if (!execUvcControl(gymbal_restart)) {
  //   std::cerr << "Failed to restart gymbal." << std::endl;
  //   return false;
  // }

  if (disable_fullHD) {
    uint8_t video_resolution_data = static_cast<uint8_t>(3);
    struct uvc_xu_control_query set_video_resolution_query = { kUnit1, 0xF, UVC_SET_CUR, 1, &video_resolution_data };
    if (!execUvcControl(set_video_resolution_query)) {
      std::cerr << "Failed to set video resolution data." << std::endl;
      return false;
    }
  }
  return true;
}

bool CxGb400::sendCopterAttitude(const double& q_w, const double& q_x, const double& q_y, const double& q_z)
{
  AttitudeMsg msg;
  msg.q_w = static_cast<int16_t>(q_w * 1.0e4);
  msg.q_x = static_cast<int16_t>(q_x * 1.0e4);
  msg.q_y = static_cast<int16_t>(q_y * 1.0e4);
  msg.q_z = static_cast<int16_t>(q_z * 1.0e4);
  union MsgUnion
  {
    AttitudeMsg msg;
    uint8_t data[8];
  } msg_union;
  msg_union.msg = msg;

  struct uvc_xu_control_query attitude_query = { kUnit3, 0x02, UVC_SET_CUR, 8, msg_union.data };
  if (!execUvcControl(attitude_query)) {
    std::cerr << "Failed to send attitude." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::fullReset()
{
  uint8_t reset = 1;
  struct uvc_xu_control_query full_reset_query = { kUnit1, 0x05, UVC_SET_CUR, 1, &reset };
  if (!execUvcControl(full_reset_query)) {
    std::cerr << "Failed to execute full reset." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::turnOffUavcan()
{
  uint8_t is_uavcan_on = 0;
  struct uvc_xu_control_query check_UAVCAN_query = { kUnit2, 0x1E, UVC_SET_CUR, 1, &is_uavcan_on };
  if (!execUvcControl(check_UAVCAN_query)) {
    std::cerr << "Failed to turn off UAVCAN." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::sendGimbalCtrl(const double& pitch_deg, const double& yaw_deg)
{
  if (pitch_deg > kPitchCmdMax) {
    std::cerr << "Pitch command is larger than the max value." << std::endl;
    return false;
  }
  if (pitch_deg < kPitchCmdMin) {
    std::cerr << "Pitch command is smaller than the minimum value." << std::endl;
    return false;
  }
  if (yaw_deg > kYawCmdMax) {
    std::cerr << "Yaw command is larger than the max value." << std::endl;
    return false;
  }
  if (yaw_deg < kYawCmdMin) {
    std::cerr << "Yaw command is smaller than the minimum value." << std::endl;
    return false;
  }
  GimbalCtrlMsg msg;
  msg.pitch_cmd_type = 0x02;
  msg.yaw_cmd_type = 0x02;
  msg.pitch_cmd_value = static_cast<int16_t>(pitch_deg * kGimbalAngleResolution);
  msg.yaw_cmd_value = static_cast<int16_t>(yaw_deg * kGimbalAngleResolution);
  union MsgUnion
  {
    GimbalCtrlMsg msg;
    uint8_t data[6];
  } msg_union;
  msg_union.msg = msg;

  struct uvc_xu_control_query gimbal_ctrl_query = { kUnit3, 0x03, UVC_SET_CUR, 6, msg_union.data };
  if (!execUvcControl(gimbal_ctrl_query)) {
    std::cerr << "Failed to send gimbal control." << std::endl;
    return false;
  }
  return true;
}

}  // namespace driver
