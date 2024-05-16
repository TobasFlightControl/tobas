#pragma once

#include <cstddef>
#include <array>
#include <string>

#include "../include/tobas_navio_core/util.hpp"

namespace navio
{
/**
 * @brief 全てのPWMチャンネルを制御するモジュール．
 *
 * @note 複数のスレッドから同時にアクセスするとバグるため，
 * 基本的には1つのノードのみがこのクラスを持つべき．
 */
class PWM
{
  static constexpr size_t kChannelCount = 14;

public:
  explicit PWM();

  bool initialize(const size_t& channel);
  bool remove(const size_t& channel);

  inline bool enable(const size_t& channel);
  inline bool disable(const size_t& channel);
  inline bool setFrequency(const size_t& channel, const size_t& freq);
  inline bool setDutyCycle(const size_t& channel, const double& period_us);

private:
  std::array<std::string, kChannelCount> enable_paths_;
  std::array<std::string, kChannelCount> period_paths_;
  std::array<std::string, kChannelCount> duty_paths_;
};

inline bool PWM::enable(const size_t& channel)
{
  return writeFile(enable_paths_[channel].c_str(), "1") >= 0;
}

inline bool PWM::disable(const size_t& channel)
{
  return writeFile(enable_paths_[channel].c_str(), "0") >= 0;
}

inline bool PWM::setFrequency(const size_t& channel, const size_t& freq)
{
  const auto period_ns = static_cast<uint32_t>(1e+9 / freq);
  return writeFile(period_paths_[channel].c_str(), "%u", period_ns) >= 0;
}

inline bool PWM::setDutyCycle(const size_t& channel, const double& period_us)
{
  const auto period_ns = static_cast<uint32_t>(period_us * 1e+3);
  return writeFile(duty_paths_[channel].c_str(), "%u", period_ns) >= 0;
}
}  // namespace navio
