#include <boost/property_tree/ini_parser.hpp>

#include <dh_std_tools/fstream.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../../include/tobas_real/rcin_calibration.hpp"
#include "../../include/tobas_real/common.hpp"

#define SLEEP_TIME 3  // [s]

using namespace std;

namespace tobas_real
{
RCInputCalibrator::RCInputCalibrator()
{
}

void RCInputCalibrator::run()
{
  rcin_.initialize();

  // Roll Neutoral
  rosInfo("Please set the ROLL lever to NEUTORAL and press Enter when ready.");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto roll_neutoral = readRCInput(kRCInputChannelRoll);

  // Roll Left
  rosInfo("Please set the ROLL lever to LEFT and press Enter when ready.");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto roll_left = readRCInput(kRCInputChannelRoll);

  // Roll Right
  rosInfo("Please set the ROLL lever to RIGHT and press Enter when ready.");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto roll_right = readRCInput(kRCInputChannelRoll);

  if (!(roll_left < roll_neutoral && roll_neutoral < roll_right))
  {
    rosthrow("Invalid value on ROLL lever. 'LEFT < NEUTORAL < RIGHT' must be satisfied.");
  }

  // Pitch Neutoral
  rosInfo("Please set the PITCH lever to NEUTORAL and press Enter when ready.");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto pitch_neutoral = readRCInput(kRCInputChannelPitch);

  // Pitch Up
  rosInfo("Please set the PITCH lever to UP and press Enter when ready.");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto pitch_up = readRCInput(kRCInputChannelPitch);

  // Pitch Down
  rosInfo("Please set the PITCH lever to DOWN and press Enter when ready.");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto pitch_down = readRCInput(kRCInputChannelPitch);

  if (!(pitch_up < pitch_neutoral && pitch_neutoral < pitch_down))
  {
    rosthrow("Invalid period on PITCH lever. 'UP < NEUTORAL < DOWN' must be satisfied.");
  }

  // Yaw Neutoral
  rosInfo("Please set the YAW lever to NEUTORAL and press Enter when ready.");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto yaw_neutoral = readRCInput(kRCInputChannelYaw);

  // Yaw Left
  rosInfo("Please set the YAW lever to LEFT and press Enter when ready.");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto yaw_left = readRCInput(kRCInputChannelYaw);

  // Yaw Right
  rosInfo("Please set the YAW lever to RIGHT and press Enter when ready.");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto yaw_right = readRCInput(kRCInputChannelYaw);

  if (!(yaw_left < yaw_neutoral && yaw_neutoral < yaw_right))
  {
    rosthrow("Invalid value on YAW lever. 'LEFT < NEUTORAL < RIGHT' must be satisfied.");
  }

  // Throttle Up
  rosInfo("Please set the THROTTLE lever to UP and press Enter when ready.");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto throttle_up = readRCInput(kRCInputChannelThrottle);

  // Throttle Down
  rosInfo("Please set the THROTTLE lever to DOWN and press Enter when ready.");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto throttle_down = readRCInput(kRCInputChannelThrottle);

  if (!(throttle_down < throttle_up))
  {
    rosthrow("Invalid period on THROTTLE lever. 'DOWN < UP' must be satisfied.");
  }

  // Switch Up
  rosInfo("Please set the SWITCH-A to UP and press Enter when ready.");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto switch_up = readRCInput(kRCInputChannelSwitch);

  // Switch Down
  rosInfo("Please set the SWITCH-A to DOWN and press Enter when ready.");
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto switch_down = readRCInput(kRCInputChannelSwitch);

  if (!(switch_up < switch_down))
  {
    rosthrow("Invalid period on SWITCH-A. 'UP < DOWN' must be satisfied.");
  }

  // Configに保存
  // 設定ファイルに係数を書き込む
  boost::property_tree::ptree pt;
  if (dh_std::fileExists(kConfigPath))
  {
    boost::property_tree::ini_parser::read_ini(kConfigPath, pt);
  }
  pt.put(kConfigKey_RcRollNeutoral, roll_neutoral);
  pt.put(kConfigKey_RcRollLeft, roll_left);
  pt.put(kConfigKey_RcRollRight, roll_right);
  pt.put(kConfigKey_RcPitchNeutoral, pitch_neutoral);
  pt.put(kConfigKey_RcPitchUp, pitch_up);
  pt.put(kConfigKey_RcPitchDown, pitch_down);
  pt.put(kConfigKey_RcYawNeutoral, yaw_neutoral);
  pt.put(kConfigKey_RcYawNeutoral, yaw_left);
  pt.put(kConfigKey_RcYawNeutoral, yaw_right);
  pt.put(kConfigKey_RcThrottleUp, throttle_up);
  pt.put(kConfigKey_RcThrottleDown, throttle_down);
  pt.put(kConfigKey_RcSwitchUp, switch_up);
  pt.put(kConfigKey_RcSwitchDown, switch_down);
  boost::property_tree::ini_parser::write_ini(kConfigPath, pt);
  rosInfo("RC input periods are saved to '" << kConfigPath << "'.");
}

double RCInputCalibrator::readRCInput(uint32_t channel)
{
  // RC入力を取得
  int period_sum = 0;
  for (uint32_t _ = 0; _ < kDataCount; ++_)
  {
    const auto period = rcin_.read(kRCInputChannelRoll);
    if (period < 0)
    {
      rosthrow("Failed to read RC input.");
    }
    rosInfoThrottle(kInfoPeriod, "Period on channel " << channel << ": " << period);
    period_sum += period;
    usleep(kSleepTime);
  }

  // 平均を計算
  const auto period_mean = static_cast<double>(period_sum) / kDataCount;
  rosInfo("Finished reading. Mean period on channel " << channel << " is: " << period_mean);
  return period_mean;
}
}  // namespace tobas_real
