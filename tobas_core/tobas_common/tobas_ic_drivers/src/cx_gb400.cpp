// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/cx_gb400.hpp"

#include <iostream>

namespace tobas
{
namespace driver
{
CxGb400::CxGb400() : linux::VideoDev::VideoDev()
{
}

bool CxGb400::initialize(
  const char* video_dev,
  const CameraPosition& camera_position,
  const bool disable_full_hd,
  const bool disable_video_streaming)
{
  if (!linux::VideoDev::initialize(video_dev, "MJPG", disable_video_streaming, 1280, 720)) {
    std::cerr << "Failed to initialize video device." << std::endl;
    return false;
  }
  // Check UAVCAN is turned off (= UVC control is available).
  uint8_t is_uavcan_on = 0;
  const uvc_xu_control_query check_uavcan_query = { kUnit2, 0x1E, UVC_GET_CUR, 1, &is_uavcan_on };
  if (!execUvcControl(check_uavcan_query)) {
    std::cerr << "Failed to check UAVCAN query." << std::endl;
    return false;
  }
  if (is_uavcan_on) {
    std::cerr << "UAVCAN is ON. Reset the camera and set UAVCAN OFF following the EXCEL file received from xacti."
              << std::endl;
    return false;
  }

  // Set camera position.
  uint8_t cam_pos_data = static_cast<uint8_t>(camera_position);
  const uvc_xu_control_query set_cam_pos_query = { kUnit2, 0x1B, UVC_SET_CUR, 1, &cam_pos_data };
  if (!execUvcControl(set_cam_pos_query)) {
    std::cerr << "Failed to set camera position." << std::endl;
    return false;
  }

  if (disable_full_hd) {
    uint8_t video_resolution_data = static_cast<uint8_t>(3);
    const uvc_xu_control_query set_video_resolution_query = { kUnit1, 0xF, UVC_SET_CUR, 1, &video_resolution_data };
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
    uint8_t data[sizeof(AttitudeMsg)];
  } msg_union;
  msg_union.msg = msg;

  const uvc_xu_control_query attitude_query = { kUnit3, 0x02, UVC_SET_CUR, sizeof(AttitudeMsg), msg_union.data };
  if (!execUvcControl(attitude_query)) {
    std::cerr << "Failed to send attitude." << std::endl;
    return false;
  }

  return true;
}

bool CxGb400::fullReset()
{
  uint8_t reset = 1;
  const uvc_xu_control_query full_reset_query = { kUnit1, 0x05, UVC_SET_CUR, 1, &reset };
  if (!execUvcControl(full_reset_query)) {
    std::cerr << "Failed to execute full reset." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::turnOffUavcan()
{
  uint8_t is_uavcan_on = 0;
  const uvc_xu_control_query check_uavcan_query = { kUnit2, 0x1E, UVC_SET_CUR, 1, &is_uavcan_on };
  if (!execUvcControl(check_uavcan_query)) {
    std::cerr << "Failed to turn off UAVCAN." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::sendGimbalCtrl(const double& pitch_deg, const double& yaw_deg)
{
  if (pitch_deg > kPitchCmdMax) {
    std::cerr << "Pitch command is larger than the maximum value." << std::endl;
    return false;
  }
  if (pitch_deg < kPitchCmdMin) {
    std::cerr << "Pitch command is smaller than the minimum value." << std::endl;
    return false;
  }
  if (yaw_deg > kYawCmdMax) {
    std::cerr << "Yaw command is larger than the maximum value." << std::endl;
    return false;
  }
  if (yaw_deg < kYawCmdMin) {
    std::cerr << "Yaw command is smaller than the minimum value." << std::endl;
    return false;
  }

  constexpr double kGimbalAngleResolution = 1.0 / 0.01;
  GimbalCtrlMsg msg;
  msg.pitch_cmd_type = 0x02;
  msg.yaw_cmd_type = 0x02;
  msg.pitch_cmd_value = static_cast<int16_t>(pitch_deg * kGimbalAngleResolution);
  msg.yaw_cmd_value = static_cast<int16_t>(yaw_deg * kGimbalAngleResolution);
  union MsgUnion
  {
    GimbalCtrlMsg msg;
    uint8_t data[sizeof(GimbalCtrlMsg)];
  } msg_union;
  msg_union.msg = msg;

  const uvc_xu_control_query gimbal_ctrl_query = { kUnit3, 0x03, UVC_SET_CUR, sizeof(GimbalCtrlMsg), msg_union.data };
  if (!execUvcControl(gimbal_ctrl_query)) {
    std::cerr << "Failed to send gimbal control." << std::endl;
    return false;
  }

  return true;
}

bool CxGb400::formatSdCard()
{
  uint8_t execute = 0;
  const uvc_xu_control_query format_sd_card_query = { kUnit2, 0x15, UVC_SET_CUR, 1, &execute };
  if (!execUvcControl(format_sd_card_query)) {
    std::cerr << "Failed to format the SD card." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::takePictureToSd()
{
  uint8_t execute = 0;
  const uvc_xu_control_query take_picture_query = { kUnit1, 0x09, UVC_SET_CUR, 1, &execute };
  if (!execUvcControl(take_picture_query)) {
    std::cerr << "Failed to take a picture." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::setPhotoQuality(const PhotoQuality& photo_quality)
{
  uint8_t photo_quality_data = static_cast<uint8_t>(photo_quality);
  const uvc_xu_control_query set_photo_quality_query = { kUnit1, 0x0B, UVC_SET_CUR, 1, &photo_quality_data };
  if (!execUvcControl(set_photo_quality_query)) {
    std::cerr << "Failed to set photo quality." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::startRecording()
{
  uint8_t start_recording = 0x01;
  const uvc_xu_control_query start_recording_query = { kUnit1, 0x0B, UVC_SET_CUR, 1, &start_recording };
  if (!execUvcControl(start_recording_query)) {
    std::cerr << "Failed to start recording." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::stopRecording()
{
  uint8_t stop_recording = 0x00;
  const uvc_xu_control_query stop_recording_query = { kUnit1, 0x0B, UVC_SET_CUR, 1, &stop_recording };
  if (!execUvcControl(stop_recording_query)) {
    std::cerr << "Failed to stop recording." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::setVideoResolution(const VideoQuality& video_quality)
{
  uint8_t video_quality_data = static_cast<uint8_t>(video_quality);
  const uvc_xu_control_query set_video_resolution = { kUnit1, 0x0F, UVC_SET_CUR, 1, &video_quality_data };
  if (!execUvcControl(set_video_resolution)) {
    std::cerr << "Failed to set video resolution." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::setVideoFrameRate(const VideoFrameRate& video_frame_rate)
{
  uint8_t video_frame_rate_data = static_cast<uint8_t>(video_frame_rate);
  const uvc_xu_control_query set_video_frame_rate = { kUnit1, 0x10, UVC_SET_CUR, 1, &video_frame_rate_data };
  if (!execUvcControl(set_video_frame_rate)) {
    std::cerr << "Failed to set video frame rate." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::getExposureMode(uint8_t& exposure_mode)
{
  const uvc_xu_control_query get_exposure_mode_query = { kUnit1, 0x14, UVC_GET_CUR, 1, &exposure_mode };
  if (!execUvcControl(get_exposure_mode_query)) {
    std::cerr << "Failed to get exposure mode." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::setExposureMode(const ExposureMode& exposure_mode)
{
  uint8_t exposure_mode_data = static_cast<uint8_t>(exposure_mode);
  const uvc_xu_control_query set_exposure_mode = { kUnit1, 0x14, UVC_SET_CUR, 1, &exposure_mode_data };
  if (!execUvcControl(set_exposure_mode)) {
    std::cerr << "Failed to set exposure mode." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::getExposureTime(uint8_t& exposure_time)
{
  const uvc_xu_control_query get_exposure_time_query = { kUnit1, 0x15, UVC_GET_CUR, 1, &exposure_time };
  if (!execUvcControl(get_exposure_time_query)) {
    std::cerr << "Failed to get exposure time." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::setExposureTime(const ExposureTime& exposure_time)
{
  uint8_t exposure_time_data = static_cast<uint8_t>(exposure_time);
  const uvc_xu_control_query set_exposure_time = { kUnit1, 0x15, UVC_SET_CUR, 1, &exposure_time_data };
  if (!execUvcControl(set_exposure_time)) {
    std::cerr << "Failed to set exposure time." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::getIsoSensitivity(uint8_t& iso_sensitivity)
{
  const uvc_xu_control_query get_iso_sensitivity_query = { kUnit1, 0x16, UVC_GET_CUR, 1, &iso_sensitivity };
  if (!execUvcControl(get_iso_sensitivity_query)) {
    std::cerr << "Failed to get iso sensitivity." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::setIsoSensitivity(const IsoSensitivity& iso_sensitivity)
{
  uint8_t iso_sensitivity_data = static_cast<uint8_t>(iso_sensitivity);
  const uvc_xu_control_query set_iso_sensivitity = { kUnit1, 0x16, UVC_SET_CUR, 1, &iso_sensitivity_data };
  if (!execUvcControl(set_iso_sensivitity)) {
    std::cerr << "Failed to set iso sensitivity." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::getExposureCompensation(uint8_t& exposure_compensation)
{
  const uvc_xu_control_query get_exposure_compensation_query = { kUnit1, 0x17, UVC_GET_CUR, 1, &exposure_compensation };
  if (!execUvcControl(get_exposure_compensation_query)) {
    std::cerr << "Failed to get exposure compensation." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::setExposureCompensation(const ExposureCompensation& exposure_compensation)
{
  uint8_t exposure_compensation_data = static_cast<uint8_t>(exposure_compensation);
  const uvc_xu_control_query set_exposure_compensation = { kUnit1, 0x17, UVC_SET_CUR, 1, &exposure_compensation_data };
  if (!execUvcControl(set_exposure_compensation)) {
    std::cerr << "Failed to set exposure compensation." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::getPhotometry(uint8_t& photometry)
{
  const uvc_xu_control_query get_photometry_query = { kUnit1, 0x18, UVC_GET_CUR, 1, &photometry };
  if (!execUvcControl(get_photometry_query)) {
    std::cerr << "Failed to get photometry." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::setPhotometry(const Photometry& photometry)
{
  uint8_t photometry_data = static_cast<uint8_t>(photometry);
  const uvc_xu_control_query set_photometry = { kUnit1, 0x18, UVC_SET_CUR, 1, &photometry_data };
  if (!execUvcControl(set_photometry)) {
    std::cerr << "Failed to set photometry." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::setWhiteBalance(const WhiteBalance& white_balance)
{
  uint8_t white_balance_data = static_cast<uint8_t>(white_balance);
  const uvc_xu_control_query set_white_balance = { kUnit1, 0x19, UVC_SET_CUR, 1, &white_balance_data };
  if (!execUvcControl(set_white_balance)) {
    std::cerr << "Failed to white balance." << std::endl;
    return false;
  }
  return true;
}

bool CxGb400::getCameraStatus(
  bool& sd_full,
  bool& time_not_set,
  bool& media_error,
  bool& lens_error,
  bool& gimbal_error,
  bool& gimbal_motor_error,
  bool& gimbal_control_error,
  bool& thermal_error,
  uint32_t& video_remain_time,
  uint32_t& photo_remain_count,
  uint32_t& card_full_size,
  uint32_t& card_free_mem,
  double& aperture,
  uint16_t& iso)
{
  union MsgUnion
  {
    CameraStatusMsg msg;
    uint8_t data[sizeof(CameraStatusMsg)];
  } msg_union;

  const uvc_xu_control_query get_camera_status_query = {
    kUnit2, 0x0B, UVC_GET_CUR, sizeof(CameraStatusMsg), msg_union.data
  };
  if (!execUvcControl(get_camera_status_query)) {
    std::cerr << "Failed to send gimbal control." << std::endl;
    return false;
  }

  // TODO: Other status values are also sent, so read them if needed.
  // They are not currently necessary.
  const uint32_t error_status = msg_union.msg.error_status;
  sd_full = interpretCameraError(error_status, kSdFull);
  time_not_set = interpretCameraError(error_status, kTimeNotSet);
  media_error = interpretCameraError(error_status, kMediaError);
  lens_error = interpretCameraError(error_status, kLensError);
  gimbal_error = interpretCameraError(error_status, kGimbalError);
  gimbal_motor_error = interpretCameraError(error_status, kGimbalMotorError);
  gimbal_control_error = interpretCameraError(error_status, kGimbalControlError);
  thermal_error = interpretCameraError(error_status, kThermalError);
  video_remain_time = msg_union.msg.video_remain_time;
  photo_remain_count = msg_union.msg.photo_remain_count;
  card_full_size = msg_union.msg.card_full_size;
  card_free_mem = msg_union.msg.card_free_mem;
  aperture = msg_union.msg.get_aperture / 100.0;
  iso = msg_union.msg.get_iso_sensitivity;
  return true;
}

bool CxGb400::interpretCameraError(const uint32_t& error_status, const CameraErrorStatusDigit& digit)
{
  // If the `digit`-th bit of `error_status` is 1, the corresponding error is active.
  return (error_status >> digit) & 1;
}

}  // namespace driver
}  // namespace tobas
