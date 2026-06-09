// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace camera
{
namespace topic
{
static constexpr char kCameraStatus[] = "camera_status";
static constexpr char kExposureCmd[] = "exposure_command";
static constexpr char kExposureStatus[] = "exposure_status";
static constexpr char kGimbalAttitudeCmd[] = "gimbal_attitude_command";
};  // namespace topic

namespace service
{
static constexpr char kFormatSdCard[] = "format_sd_card";
static constexpr char kSetPhotoQuality[] = "set_photo_quality";
static constexpr char kSetVideoFrameRate[] = "set_video_frame_rate";
static constexpr char kSetVideoQuality[] = "set_video_quality";
static constexpr char kStartRecording[] = "start_recording";
static constexpr char kStopRecording[] = "stop_recording";
static constexpr char kTakePictureToSd[] = "take_picture_to_sd";
}  // namespace service
}  // namespace camera
}  // namespace tobas
