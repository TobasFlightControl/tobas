#include <iostream>
#include <boost/property_tree/ini_parser.hpp>

#include <tobas_std_tools/fstream.hpp>
#include <tobas_std_tools/console.hpp>

#include "../../include/tobas_real/calibration/rcin_calibration.hpp"
#include "../../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
RCInputCalibrator::RCInputCalibrator()
{
  rcin_.initialize();
}

void RCInputCalibrator::run()
{
  double roll_left, roll_right;
  double pitch_up, pitch_down;
  double yaw_left, yaw_right;
  double thrust_up, thrust_down;
  double estop_up, estop_down;
  double gpsw_up, gpsw_down;
  int num_modes;
  vector<double> modes;

  // Roll
  while (true)
  {
    // Left
    cout << "Please set the ROLL (CH" << kRcChannelRoll + 1 << ") lever to LEFT and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    roll_left = readRCInput(kRcChannelRoll);

    // Right
    cout << "Please set the ROLL (CH" << kRcChannelRoll + 1 << ") lever to RIGHT and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    roll_right = readRCInput(kRcChannelRoll);

    if (abs(roll_left - roll_right) < kPeriodDiffThreshold)
    {
      TOBAS_ERROR("The signals on Roll channel are too close. Please retry.");
      continue;
    }

    break;
  }

  // Pitch
  while (true)
  {
    // Up
    cout << "Please set the PITCH (CH" << kRcChannelPitch + 1 << ") lever to UP and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    pitch_up = readRCInput(kRcChannelPitch);

    // Down
    cout << "Please set the PITCH (CH" << kRcChannelPitch + 1 << ") lever to DOWN and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    pitch_down = readRCInput(kRcChannelPitch);

    if (abs(pitch_up - pitch_down) < kPeriodDiffThreshold)
    {
      TOBAS_ERROR("The signals on Pitch channel are too close. Please retry.");
      continue;
    }

    break;
  }

  // Yaw
  while (true)
  {
    // Left
    cout << "Please set the YAW (CH" << kRcChannelYaw + 1 << ") lever to LEFT and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    yaw_left = readRCInput(kRcChannelYaw);

    // Right
    cout << "Please set the YAW (CH" << kRcChannelYaw + 1 << ") lever to RIGHT and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    yaw_right = readRCInput(kRcChannelYaw);

    if (abs(yaw_left - yaw_right) < kPeriodDiffThreshold)
    {
      TOBAS_ERROR("The signals on Yaw channel are too close. Please retry.");
      continue;
    }

    break;
  }

  // Thrust
  while (true)
  {
    // Up
    cout << "Please set the THRUST (CH" << kRcChannelThrust + 1 << ") lever to UP and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    thrust_up = readRCInput(kRcChannelThrust);

    // Down
    cout << "Please set the THRUST (CH" << kRcChannelThrust + 1
         << ") lever to DOWN and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    thrust_down = readRCInput(kRcChannelThrust);

    if (abs(thrust_up - thrust_down) < kPeriodDiffThreshold)
    {
      TOBAS_ERROR("The signals on Thrust channel are too close. Please retry.");
      continue;
    }

    break;
  }

  // E-Stop
  while (true)
  {
    // Up
    cout << "Please set the E-STOP (CH" << kRcChannelEStop + 1 << ") to UP and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    estop_up = readRCInput(kRcChannelEStop);

    // Down
    cout << "Please set the E-STOP (CH" << kRcChannelEStop + 1 << ") to DOWN and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    estop_down = readRCInput(kRcChannelEStop);

    if (abs(estop_up - estop_down) < kPeriodDiffThreshold)
    {
      TOBAS_ERROR("The signals on E-Stop channel are too close. Please retry.");
      continue;
    }

    break;
  }

  // GPSw (General Purpose Switch)
  while (true)
  {
    // Up
    cout << "Please set the GPSw (CH" << kRcChannelGPSw + 1 << ") to UP and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    gpsw_up = readRCInput(kRcChannelGPSw);

    // Down
    cout << "Please set the GPSw (CH" << kRcChannelGPSw + 1 << ") to DOWN and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    gpsw_down = readRCInput(kRcChannelGPSw);

    if (abs(gpsw_up - gpsw_down) < kPeriodDiffThreshold)
    {
      TOBAS_ERROR("The signals on GPSw channel are too close. Please retry.");
      continue;
    }

    break;
  }

  // Mode
  while (true)
  {
    cout << "Please enter the number of flight modes: ";
    cin >> num_modes;
    cin.ignore(numeric_limits<streamsize>::max(), '\n');  // ストリームに残っている改行を処理
    if (num_modes <= 0)
    {
      TOBAS_ERROR("The number of flight modes must be positive. Please retry.");
      continue;
    }
    else if (static_cast<size_t>(num_modes) > kMaxNrOfFlightModes)
    {
      TOBAS_ERROR("The number of flight modes is too large. Please retry.");
      continue;
    }
    break;
  }

  modes.resize(num_modes);
  while (true)
  {
    for (int i = 0; i < num_modes; ++i)
    {
      cout << "Please set the MODE (CH" << kRcChannelMode + 1 << ") to " << i + 1
           << " and press Enter:";
      cin.ignore(numeric_limits<streamsize>::max(), '\n');
      modes[i] = readRCInput(kRcChannelMode);
    }

    if (isDifferentModesTooClose(modes))
    {
      TOBAS_ERROR("The signals of different modes are too close. Please retry.");
      continue;
    }

    break;
  }

  // Configに保存
  boost::property_tree::ptree pt;
  if (tobas_std::fileExists(kConfigPath))
  {
    boost::property_tree::ini_parser::read_ini(kConfigPath, pt);
  }

  pt.put(kConfigKey_RcRollLeft, roll_left);
  pt.put(kConfigKey_RcRollRight, roll_right);
  pt.put(kConfigKey_RcPitchUp, pitch_up);
  pt.put(kConfigKey_RcPitchDown, pitch_down);
  pt.put(kConfigKey_RcYawLeft, yaw_left);
  pt.put(kConfigKey_RcYawRight, yaw_right);
  pt.put(kConfigKey_RcThrustUp, thrust_up);
  pt.put(kConfigKey_RcThrustDown, thrust_down);
  pt.put(kConfigKey_RcEStopOn, estop_up);
  pt.put(kConfigKey_RcEStopOff, estop_down);
  pt.put(kConfigKey_RcGPSwOn, gpsw_up);
  pt.put(kConfigKey_RcGPSwOff, gpsw_down);

  pt.put(kConfigKey_RcNrOfModes, num_modes);
  for (int i = 0; i < num_modes; ++i)
  {
    const string key = kConfigKey_RcModePrefix + to_string(i);
    pt.put(key, modes[i]);
  }

  boost::property_tree::ini_parser::write_ini(kConfigPath, pt);
  cout << "Calibration finished. The result is saved to '" << kConfigPath << "'." << endl;
}

double RCInputCalibrator::readRCInput(const size_t& channel)
{
  // RC入力を取得
  size_t period_sum = 0;
  for (size_t _ = 0; _ < kDataCount; ++_)
  {
    const auto period = rcin_.read(channel);
    if (period <= 0)
    {
      throw runtime_error("Failed to read RC input.");
    }
    period_sum += period;
    usleep(kSleepTime);
  }

  // 平均を計算
  const auto period_mean = static_cast<double>(period_sum) / kDataCount;
  cout << "Finished reading. Mean period on CH" << channel + 1 << " is: " << period_mean << endl;
  return period_mean;
}

bool RCInputCalibrator::isDifferentModesTooClose(const vector<double>& modes) const
{
  for (size_t i = 0; i < modes.size(); ++i)
    for (size_t j = i + 1; j < modes.size(); ++j)
      if (abs(modes[i] - modes[j]) < kPeriodDiffThreshold)
        return true;

  return false;
}
}  // namespace tobas_real
