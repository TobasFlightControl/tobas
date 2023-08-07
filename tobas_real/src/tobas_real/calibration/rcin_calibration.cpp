#include <boost/property_tree/ini_parser.hpp>

#include <dh_std_tools/fstream.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../../../include/tobas_real/calibration/rcin_calibration.hpp"
#include "../../../include/tobas_real/common.hpp"

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
  cout << "Please set the ROLL lever to NEUTORAL and press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto roll_neutoral = readRCInput(kRCInputChannelRoll);

  // Roll Left
  cout << "Please set the ROLL lever to LEFT and press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto roll_left = readRCInput(kRCInputChannelRoll);

  // Roll Right
  cout << "Please set the ROLL lever to RIGHT and press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto roll_right = readRCInput(kRCInputChannelRoll);

  if (!(roll_left + kPeriodMargin < roll_neutoral && roll_neutoral + kPeriodMargin < roll_right))
  {
    rosthrow("Invalid value on ROLL lever. 'LEFT < NEUTORAL < RIGHT' must be satisfied.");
  }

  // Pitch Neutoral
  cout << "Please set the PITCH lever to NEUTORAL and press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto pitch_neutoral = readRCInput(kRCInputChannelPitch);

  // Pitch Up
  cout << "Please set the PITCH lever to UP and press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto pitch_up = readRCInput(kRCInputChannelPitch);

  // Pitch Down
  cout << "Please set the PITCH lever to DOWN and press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto pitch_down = readRCInput(kRCInputChannelPitch);

  if (!(pitch_up + kPeriodMargin < pitch_neutoral && pitch_neutoral + kPeriodMargin < pitch_down))
  {
    rosthrow("Invalid period on PITCH lever. 'UP < NEUTORAL < DOWN' must be satisfied.");
  }

  // Yaw Neutoral
  cout << "Please set the YAW lever to NEUTORAL and press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto yaw_neutoral = readRCInput(kRCInputChannelYaw);

  // Yaw Left
  cout << "Please set the YAW lever to LEFT and press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto yaw_left = readRCInput(kRCInputChannelYaw);

  // Yaw Right
  cout << "Please set the YAW lever to RIGHT and press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto yaw_right = readRCInput(kRCInputChannelYaw);

  if (!(yaw_left + kPeriodMargin < yaw_neutoral && yaw_neutoral + kPeriodMargin < yaw_right))
  {
    rosthrow("Invalid value on YAW lever. 'LEFT < NEUTORAL < RIGHT' must be satisfied.");
  }

  // Thrust Up
  cout << "Please set the THRUST lever to UP and press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto thrust_up = readRCInput(kRCInputChannelThrust);

  // Thrust Down
  cout << "Please set the THRUST lever to DOWN and press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto thrust_down = readRCInput(kRCInputChannelThrust);

  if (!(thrust_down + kPeriodMargin < thrust_up))
  {
    rosthrow("Invalid period on THRUST lever. 'DOWN < UP' must be satisfied.");
  }

  // Toggle Up
  cout << "Please set the TOGGLE-A to UP and press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto toggle_up = readRCInput(kRCInputChannelToggle);

  // Toggle Down
  cout << "Please set the TOGGLE-A to DOWN and press Enter:";
  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  const auto toggle_down = readRCInput(kRCInputChannelToggle);

  if (!(toggle_up + kPeriodMargin < toggle_down))
  {
    rosthrow("Invalid period on TOGGLE-A. 'UP < DOWN' must be satisfied.");
  }

  // Configに保存
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
  pt.put(kConfigKey_RcYawLeft, yaw_left);
  pt.put(kConfigKey_RcYawRight, yaw_right);
  pt.put(kConfigKey_RcThrustUp, thrust_up);
  pt.put(kConfigKey_RcThrustDown, thrust_down);
  pt.put(kConfigKey_RcToggleUp, toggle_up);
  pt.put(kConfigKey_RcToggleDown, toggle_down);
  boost::property_tree::ini_parser::write_ini(kConfigPath, pt);
  rosInfo("Calibration finished. The result is saved to '" << kConfigPath << "'.");
}

double RCInputCalibrator::readRCInput(uint32_t channel)
{
  // RC入力を取得
  int period_sum = 0;
  for (uint32_t _ = 0; _ < kDataCount; ++_)
  {
    const auto period = rcin_.read(channel);
    if (period < 0)
    {
      rosthrow("Failed to read RC input.");
    }
    rosInfoThrottle(kShowSensorReadingPeriod, "Period on channel " << channel << ": " << period);
    period_sum += period;
    usleep(kSleepTime);
  }

  // 平均を計算
  const auto period_mean = static_cast<double>(period_sum) / kDataCount;
  rosInfo("Finished reading. Mean period on channel " << channel << " is: " << period_mean);
  return period_mean;
}
}  // namespace tobas_real
