#pragma once

namespace camera
{
namespace topic
{
static constexpr char kCameraStatusTopic[] = "camera_status";
static constexpr char kGimbalAttitudeCmdTopic[] = "gimbal_attitude_command";
};  // namespace topic

namespace service
{
static constexpr char kFormatSdCardSrv[] = "format_sd_card";
static constexpr char kSetPhotoQualitySrv[] = "set_photo_quality";
static constexpr char kSetVideoFrameRateSrv[] = "set_video_frame_rate";
static constexpr char kSetVideoQualitySrv[] = "set_video_quality";
static constexpr char kStartRecordingSrv[] = "start_recording";
static constexpr char kStopRecordingSrv[] = "stop_recording";
static constexpr char kTakePictureToSdSrv[] = "take_picture_to_sd";
}  // namespace service
}  // namespace camera
