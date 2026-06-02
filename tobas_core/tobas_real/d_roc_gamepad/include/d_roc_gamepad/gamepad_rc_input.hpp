// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

struct libevdev;

namespace tobas
{
namespace driver
{

/** @brief 飛行モード． */
enum class FlightMode : std::uint8_t
{
  kAcrobat = 0,
  kStabilize = 1,
  kLoiter = 2,
  kCount,
};

/** @brief ゲームパッド入力から生成したRC入力状態． */
struct GamepadRcInputState
{
  /* 入力デバイスを正常に読めているか． */
  bool ok = false;
  /* ロール指令 [-1, 1]． */
  double roll = 0.0;
  /* ピッチ指令 [-1, 1]． */
  double pitch = 0.0;
  /* スロットル指令 [-1, 1]． */
  double throttle = 0.0;
  /* ヨー指令 [-1, 1]． */
  double yaw = 0.0;
  /* 飛行モード． */
  std::uint8_t mode = 0;
  /* サブモード切り替え． */
  bool sub_mode = false;
  /* RC入力の有効化スイッチ． */
  bool enable = false;
  /* キルスイッチ． */
  bool kill = false;
  /* 汎用スイッチ． */
  std::array<bool, 8> gpsw = {};
};

/** @brief ゲームパッド入力をRC入力へ変換するための設定． */
struct GamepadRcInputConfig
{
  /* 軸入力を0とみなす不感帯． */
  double deadzone = 0.05;
  /* ロール軸を反転する． */
  bool invert_roll = false;
  /* ピッチ軸を反転する． */
  bool invert_pitch = true;
  /* スロットル軸を反転する． */
  bool invert_throttle = true;
  /* ヨー軸を反転する． */
  bool invert_yaw = true;
};

/**
 * @brief libevdevでゲームパッド入力を読み，RC入力状態に変換するドライバ．
 *
 * Linux input event deviceを読み取り，ボタン入力をスイッチに，絶対軸入力を正規化した
 * RC指令値に変換する．
 */
class GamepadRcInput
{
public:
  struct LibevdevDeleter
  {
    void operator()(libevdev* _dev) const;
  };

  explicit GamepadRcInput(GamepadRcInputConfig _config = {});
  GamepadRcInput(const GamepadRcInput& _other) = delete;
  GamepadRcInput(GamepadRcInput&& _other) = delete;
  GamepadRcInput& operator=(const GamepadRcInput& _other) = delete;
  GamepadRcInput& operator=(GamepadRcInput&& _other) = delete;
  ~GamepadRcInput();

  /** @brief 入力デバイスを開いて読み取り可能な状態にする． */
  bool initialize(const std::string& _device_path);
  /** @brief 入力デバイスを閉じる． */
  void close();
  /** @brief 入力デバイスが開かれているかを返す． */
  bool isOpen() const;

  /** @brief 現在のRC入力状態を読み出す． */
  bool read(GamepadRcInputState& _rc_input);

private:
  /** @brief 正負それぞれの最大値で軸入力を[-1, 1]に正規化する． */
  static double normalizeSymmetric(int _value, int _minimum, int _maximum);

  bool openDevice();
  bool poll();
  double normalizeAbs(int _code, int _value, bool _invert) const;
  void applyButton(int _code, int _value);
  void applyAbs(int _code, int _value);

  GamepadRcInputConfig config_;
  std::string device_path_;
  GamepadRcInputState rc_input_;
  int fd_ = -1;
  std::unique_ptr<libevdev, LibevdevDeleter> dev_ = {};
};
}  // namespace driver
}  // namespace tobas
