#pragma once

#include <chrono>

#include <tobas_linux/video_dev.hpp>

#define PACKED __attribute__((__packed__))  // 構造体のメンバ変数がメモリ上で連続する

namespace driver
{
/**
 * @brief A linux driver of cx_gb100 camera.
 */
class CxGb400 : public linux::VideoDev
{
public:
  static constexpr std::chrono::milliseconds kSendAttitudeInterval = std::chrono::milliseconds(200);
  enum CameraPosition : uint8_t
  {
    kLower = 0x0,         // 機体に下向きで取り付ける（レンズより台が上になる向き）
    kUpper = 0x1,         // 機体に上向きで取り付ける（レンズより台が下になる向き）
    kUpperYawFix = 0x02,  // 機体に上向きで取り付ける（ヨーは回転フリー）
    kAuto = 0x03,         // 自動で決定（デフォルト）
  };
  enum PhotoQuality : uint8_t
  {
    kSuperFine = 0x00,  // 9M (3600 x 2400)
    kFine = 0x01,       // 8M (3264 x 2448)
    kNormal = 0x02,     // 4M (2400 x 1600)
  };
  enum VideoQuality : uint8_t
  {
    k4K = 0x00,    // 4K
    k2_7K = 0x01,  // 2.7K
    kFHD = 0x02,   // Full HD
    kHD = 0x03,    // HD
  };
  enum VideoFrameRate : uint8_t
  {
    k30p = 0x00,  // 30FPS
    k60p = 0x01,  // 60FPS
  };

  explicit CxGb400();
  ~CxGb400();

  bool initialize(
    const char* video_dev,
    const CameraPosition& camera_position = kAuto,
    const bool disable_full_hd = true,
    const bool disable_video_streaming = false);

  bool sendCopterAttitude(const double& q_w, const double& q_x, const double& q_y, const double& q_z);

  /**
   * @brief Send gimbal attitude control command.
   * Specify angles in degrees satisfying -115.0 < pitch_deg < 45.0, -85.0 < yaw_deg < 85.0.
   */
  bool sendGimbalCtrl(const double& pitch_deg, const double& yaw_deg);

  /* 工場出荷リセット． */
  bool fullReset();

  /* 工場出荷リセット実施後，通常起動前に1回のみ実施する． */
  bool turnOffUavcan();

  /* SDカードをフォーマットする */
  bool formatSdCard();
  /* 静止画を撮影する. 現在設定の画質で撮影. 画像は内蔵SDカードに保存 */
  bool takePictureToSd();
  /* 静止画の画質を設定する. superfine, fine, normalの中から選択 */
  bool setPhotoQuality(const PhotoQuality& photo_quality);
  /* 動画撮影を開始する 動画はSDへ保存 */
  bool startRecording();
  /* 動画撮影を止める 動画はSDへ保存 */
  bool stopRecording();
  /* レコーディングする動画の画質を設定する */
  bool setVideoResolution(const VideoQuality& video_quality);
  /* レコーディングする動画のフレームレートを設定する */
  bool setVideoFrameRate(const VideoFrameRate& video_frame_rate);
  /* カメラ状態を取得する 結果から引数として与えられた変数の値を変更する */
  bool getCameraStatus(
    bool& sd_full,
    bool& time_not_set,
    bool& media_error, // SDカードが挿入されていないか満タン
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
    uint16_t& iso);

private:
  static constexpr double kPitchCmdMax = 45.0;    // [deg]
  static constexpr double kPitchCmdMin = -115.0;  // [deg]
  static constexpr double kYawCmdMax = 85.0;      // [deg]
  static constexpr double kYawCmdMin = -85.0;     // [deg]
  static constexpr double kGimbalAngleResolution = 1.0 / 0.01;

  enum UnitId : uint8_t
  {
    kUnit1 = 0x6,
    kUnit2 = 0x7,
    kUnit3 = 0x8,
  };

  // 0-indexed
  enum CameraErrorStatusDigit : uint8_t
  {
    kTakingPicture = 2,
    kTakingMovie = 3,
    kSdFull = 5,
    kTimeNotSet = 16,
    kMediaError = 17,
    kLensError = 18,
    kGimbalError = 20,
    kGimbalMotorError = 21,
    kGimbalControlError = 22,
    kThermalError = 24,
  };

  struct PACKED AttitudeMsg
  {
    int16_t q_w;
    int16_t q_x;
    int16_t q_y;
    int16_t q_z;
  };
  static_assert(sizeof(AttitudeMsg) == 8, "AttitudeMsg size is strange!");

  struct PACKED GimbalCtrlMsg
  {
    uint8_t pitch_cmd_type;
    uint8_t yaw_cmd_type;
    int16_t pitch_cmd_value;
    int16_t yaw_cmd_value;
  };
  static_assert(sizeof(GimbalCtrlMsg) == 6, "GimbalCtrlMsg size of strange!");

  struct PACKED CameraStatusMsg
  {
    uint32_t error_status;
    uint32_t video_remain_time;
    uint32_t photo_remain_count;
    uint32_t card_full_size;  // SDカードのサイズ 単位はMB
    uint32_t card_free_mem;   // SDカードの残りのサイズ 単位はMB
    uint16_t ad_value_body;
    uint16_t ad_value_cmos;
    uint16_t ad_value_gimbal_pitch;
    uint16_t ad_value_gimbal_roll;
    uint16_t ad_value_gimbal_tilt;
    uint16_t reserve1;
    uint8_t get_date_time[7];
    uint8_t reserve2;
    uint32_t get_exposure_time;  // 単位はus(micro second)
    uint16_t get_aperture;       // F値を100倍した値
    uint16_t get_iso_sensitivity;
  };
  static_assert(sizeof(CameraStatusMsg) == 48, "CameraStatusMsg size is strange!");

  bool interpretCameraError(const uint32_t& error_status, const CameraErrorStatusDigit& digit);
};

}  // namespace driver
