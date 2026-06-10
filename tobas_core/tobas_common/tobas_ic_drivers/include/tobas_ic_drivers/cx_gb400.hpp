// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

#include <tobas_linux/video_dev.hpp>

#define PACKED __attribute__((__packed__))  // 構造体のメンバ変数がメモリ上で連続する

namespace tobas
{
namespace driver
{
/**
 * @brief A linux driver of cx_gb400 camera.
 */
class CxGb400 : public linux::VideoDev
{
public:
  static constexpr std::chrono::milliseconds kSendAttitudeInterval = std::chrono::milliseconds(200);
  static constexpr double kPitchCmdMax = 45.0;    // [deg]
  static constexpr double kPitchCmdMin = -115.0;  // [deg]
  static constexpr double kYawCmdMax = 85.0;      // [deg]
  static constexpr double kYawCmdMin = -85.0;     // [deg]

  enum class CameraPosition : uint8_t
  {
    kLower = 0x0,         // 機体に下向きで取り付ける（レンズより台が上になる向き）
    kUpper = 0x1,         // 機体に上向きで取り付ける（レンズより台が下になる向き）
    kUpperYawFix = 0x02,  // 機体に上向きで取り付ける（ヨーは回転フリー）
    kAuto = 0x03,         // 自動で決定（デフォルト）
  };
  enum class PhotoQuality : uint8_t
  {
    kSuperFine = 0x00,  // 9M (3600 x 2400)
    kFine = 0x01,       // 8M (3264 x 2448)
    kNormal = 0x02,     // 4M (2400 x 1600)
  };
  enum class VideoQuality : uint8_t
  {
    k4K = 0x00,    // 4K
    k2_7K = 0x01,  // 2.7K
    kFHD = 0x02,   // Full HD
    kHD = 0x03,    // HD
  };
  enum class VideoFrameRate : uint8_t
  {
    k30p = 0x00,  // 30FPS
    k60p = 0x01,  // 60FPS
  };
  // 露出モードの設定, 以下の説明には多くの推測が含まれる
  // manual : 露出時間, ISO感度, 絞りを人間が指定
  // setExposureTime, setIsoSensitivity, setApertureで露出時間 & ISO感度 & 絞りの生値を設定することで撮影configを設定
  // program : 露出時間, ISO露出, 絞りをカメラが決定
  // setExposureCompensation, setPhotometryにより露出補正, 測光モード設定を行うことで間接的に露出時間, ISO感度, 絞りを変更, 撮影configを設定
  // 注 : ホワイトバランスの設定はこれとは関係ない
  enum class ExposureMode: uint8_t
  {
    kManual = 0x00,   // manual, 露出時間, ISO感度, 絞りを人間が指定
    kProgram = 0x03,  // program (default), 露出時間, ISO露出をカメラが決定(多分), 補正のためには露出補正(+0.3とか)を行う
  };
  enum class ExposureTime : uint8_t
  {
    k1_8000 = 0x00, // 1/8000
    k1_6400 = 0x01, // 1/6400
    k1_5000 = 0x02, // 1/5000
    k1_4000 = 0x03, // 1/4000
    k1_3200 = 0x04, // 1/3200
    k1_2500 = 0x05, // 1/2500
    k1_2000 = 0x06, // 1/2000
    k1_1600 = 0x07, // 1/1600
    k1_1250 = 0x08, // 1/1250
    k1_1000 = 0x09, // 1/1000
    k1_800  = 0x0A, // 1/800
    k1_640  = 0x0B, // 1/640
    k1_500  = 0x0C, // 1/500
    k1_400  = 0x0D, // 1/400
    k1_320  = 0x0E, // 1/320
    k1_250  = 0x0F, // 1/250
    k1_200  = 0x10, // 1/200
    k1_160  = 0x11, // 1/160
    k1_125  = 0x12, // 1/125
    k1_100  = 0x13, // 1/100 (default)
    k1_80   = 0x14, // 1/80
    k1_60   = 0x15, // 1/60
    k1_50   = 0x16, // 1/50
    k1_40   = 0x17, // 1/40
    k1_30   = 0x18, // 1/30
    k1_25   = 0x19, // 1/25
    k1_20   = 0x1A, // 1/20
    k1_15   = 0x1B, // 1/15
    k1_13   = 0x1C, // 1/13
    k1_10   = 0x1D, // 1/10
    k1_8    = 0x1E, // 1/8
    k1_6    = 0x1F, // 1/6
    k1_5    = 0x20, // 1/5
    k1_4    = 0x21, // 1/4
    k1_3    = 0x22, // 1/3
    k1_2p5  = 0x23, // 1/2.5 (2 point 5)
    k1_2    = 0x24, // 1/2
    k1_1p6  = 0x25, // 1/1.6
    k1_1p3  = 0x26, // 1/1.3
    k1      = 0x27, // 1
    k1p3    = 0x28, // 1.3
    k1p6    = 0x29, // 1.6
    k2      = 0x2A, // 2
    k2p5    = 0x2B, // 2.5
    k3p2    = 0x2C, // 3.2
    k4      = 0x2D, // 4
    k5      = 0x2E, // 5
    k6      = 0x2F, // 6
    k8      = 0x30, // 8
  };
  enum class IsoSensitivity : uint8_t
  {
    kAuto = 0x00, // Auto
    k125  = 0x01, // 125 (default)
    k160  = 0x02, // 160
    k200  = 0x03, // 200
    k250  = 0x04, // 250
    k320  = 0x05, // 320
    k400  = 0x06, // 400
    k500  = 0x07, // 500
    k640  = 0x08, // 640
    k800  = 0x09, // 800
    k1000 = 0x0A, // 1000
    k1250 = 0x0B, // 1250
    k1600 = 0x0C, // 1600
    k2000 = 0x0D, // 2000
    k2500 = 0x0E, // 2500
    k3200 = 0x0F, // 3200
    k4000 = 0x10, // 4000
    k5000 = 0x11, // 5000
    k6400 = 0x12, // 6400
  };
  enum class ExposureCompensation : uint8_t
  {
    k_2p0 = 0x00, // -2.0 (minus 2 point 0)
    k_1p7 = 0x01, // -1.7
    k_1p3 = 0x02, // -1.3
    k_1p0 = 0x03, // -1.0
    k_0p7 = 0x04, // -0.7
    k_0p3 = 0x05, // -0.3
    k0p0  = 0x06, // 0.0 (default)
    k0p3  = 0x07, // +0.3
    k0p7  = 0x08, // +0.7
    k1p0  = 0x09, // +1.0
    k1p3  = 0x0A, // +1.3
    k1p7  = 0x0B, // +1.7
    k2p0  = 0x0C, // +2.0
  };
  enum class Photometry : uint8_t
  {
    kMulti  = 0x00, // Multi (default)
    kCenter = 0x01, // Center
  };
  enum class WhiteBalance : uint8_t
  {
    kAuto = 0x00,           // Auto (default)
    kDaylight = 0x01,       // Daylight
    kCloudy = 0x02,         // Cloudy
    kShady = 0x03,          // Shady
    kIncandescent = 0x04,   // Incandescent
    kFluorescent = 0x05,    // Fluorescent
  };
  enum class Aperture : uint8_t
  {
    kF2p8  = 0x00, // F2.8 (2 point 8) (default)
    kF3p2  = 0x01, // F3.2
    kF3p5  = 0x02, // F3.5
    kF4p0  = 0x03, // F4.0
    kF4p5  = 0x04, // F4.5
    kF5p0  = 0x05, // F5.0
    kF5p6  = 0x06, // F5.6
    kF6p3  = 0x07, // F6.3
    kF7p1  = 0x08, // F7.1
    kF8p0  = 0x09, // F8.0
    kF9p0  = 0x0A, // F9.0
    kF10p0 = 0x0B, // F10.0
    kF11p0 = 0x0C, // F11.0
  };

  explicit CxGb400();
  ~CxGb400();

  bool initialize(
    const char* video_dev,
    const CameraPosition& camera_position = CameraPosition::kAuto,
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
  /* 露出モードを取得する 結果から与えられた変数の値を変更する */
  bool getExposureMode(uint8_t& exposure_mode);
  /* 露出モードを設定する */
  bool setExposureMode(const ExposureMode& exposure_mode);
  /* 露出時間を取得する 結果から与えられた変数の値を変更する */
  bool getExposureTime(uint8_t& exposure_time);
  /* 露出時間を設定する */
  bool setExposureTime(const ExposureTime& exposure_time);
  /* isoを取得する 結果から引数として与えられた変数の値を変更する */
  bool getIsoSensitivity(uint8_t& iso_sensitivity);
  /* isoを設定する */
  bool setIsoSensitivity(const IsoSensitivity& iso_sensitivity);
  /* 露出補正を取得する 結果から引数として与えられた変数の値を変更する:*/
  bool getExposureCompensation(uint8_t& exposure_compenstation);
  /* 露出補正を行う */
  bool setExposureCompensation(const ExposureCompensation& exposure_compensation);
  /* 測光モードを取得する 結果から引数として与えられた変数の値を変更する */
  bool getPhotometry(uint8_t& photometry);
  /* 測光モード設定を行う */
  bool setPhotometry(const Photometry& photometry);
  /* ホワイトバランスを設定する */
  bool setWhiteBalance(const WhiteBalance& white_balance);
  /* 絞りを取得する 結果から引数として与えられた変数の値を変更する */
  bool getAperture(uint8_t& aperture);
  /* 絞りを設定する */
  bool setAperture(const Aperture& aperture);
  /* カメラ状態を取得する 結果から引数として与えられた変数の値を変更する */
  bool getCameraStatus(
    bool& sd_full,
    bool& time_not_set,
    bool& media_error,  // SDカードが挿入されていないか満タン
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
}  // namespace tobas

#undef PACKED
