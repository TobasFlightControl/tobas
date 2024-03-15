#include <tobas_std_tools/property_tree.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_navio_ros/common.hpp>

#include "../include/tobas_calibration_core/rcin_calibration.hpp"

using namespace std;

namespace tobas_calibration
{
RCInputCalibrator::RCInputCalibrator()
{
  if (rcin_.initialize() < 0)
    throw runtime_error("Failed to initialize RC input driver.");
}

void RCInputCalibrator::run()
{
  double roll_left, roll_right;
  double pitch_up, pitch_down;
  double yaw_left, yaw_right;
  double thrust_up, thrust_down;
  double mode_program, mode_stabilize, mode_acrobat;
  double estop_up, estop_down;
  double gpsw_up, gpsw_down;

  // Roll
  while (true)
  {
    // Left
    cout << "Please set the ROLL (CH" << tobas_navio_ros::kRcChannelRoll + 1
         << ") lever to LEFT and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    roll_left = readRCInput(tobas_navio_ros::kRcChannelRoll);

    // Right
    cout << "Please set the ROLL (CH" << tobas_navio_ros::kRcChannelRoll + 1
         << ") lever to RIGHT and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    roll_right = readRCInput(tobas_navio_ros::kRcChannelRoll);

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
    cout << "Please set the PITCH (CH" << tobas_navio_ros::kRcChannelPitch + 1
         << ") lever to UP and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    pitch_up = readRCInput(tobas_navio_ros::kRcChannelPitch);

    // Down
    cout << "Please set the PITCH (CH" << tobas_navio_ros::kRcChannelPitch + 1
         << ") lever to DOWN and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    pitch_down = readRCInput(tobas_navio_ros::kRcChannelPitch);

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
    cout << "Please set the YAW (CH" << tobas_navio_ros::kRcChannelYaw + 1
         << ") lever to LEFT and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    yaw_left = readRCInput(tobas_navio_ros::kRcChannelYaw);

    // Right
    cout << "Please set the YAW (CH" << tobas_navio_ros::kRcChannelYaw + 1
         << ") lever to RIGHT and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    yaw_right = readRCInput(tobas_navio_ros::kRcChannelYaw);

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
    cout << "Please set the THRUST (CH" << tobas_navio_ros::kRcChannelThrust + 1
         << ") lever to UP and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    thrust_up = readRCInput(tobas_navio_ros::kRcChannelThrust);

    // Down
    cout << "Please set the THRUST (CH" << tobas_navio_ros::kRcChannelThrust + 1
         << ") lever to DOWN and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    thrust_down = readRCInput(tobas_navio_ros::kRcChannelThrust);

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
    cout << "Please set the E-STOP (CH" << tobas_navio_ros::kRcChannelEStop + 1
         << ") to UP and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    estop_up = readRCInput(tobas_navio_ros::kRcChannelEStop);

    // Down
    cout << "Please set the E-STOP (CH" << tobas_navio_ros::kRcChannelEStop + 1
         << ") to DOWN and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    estop_down = readRCInput(tobas_navio_ros::kRcChannelEStop);

    if (abs(estop_up - estop_down) < kPeriodDiffThreshold)
    {
      TOBAS_ERROR("The signals on E-Stop channel are too close. Please retry.");
      continue;
    }

    break;
  }

  // Mode
  while (true)
  {
    // Program
    cout << "Please set the Mode (CH" << tobas_navio_ros::kRcChannelMode + 1
         << ") to Program and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    mode_program = readRCInput(tobas_navio_ros::kRcChannelMode);

    // Stabilize
    cout << "Please set the Mode (CH" << tobas_navio_ros::kRcChannelMode + 1
         << ") to Stabilize and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    mode_stabilize = readRCInput(tobas_navio_ros::kRcChannelMode);

    // Acrobat
    cout << "Please set the Mode (CH" << tobas_navio_ros::kRcChannelMode + 1
         << ") to Acrobat and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    mode_acrobat = readRCInput(tobas_navio_ros::kRcChannelMode);

    if (
      abs(mode_program - mode_stabilize) < kPeriodDiffThreshold
      || abs(mode_stabilize - mode_acrobat) < kPeriodDiffThreshold
      || abs(mode_acrobat - mode_program) < kPeriodDiffThreshold)
    {
      TOBAS_ERROR("The signals on Mode channel are too close. Please retry.");
      continue;
    }

    break;
  }

  // GPSw (General Purpose Switch)
  while (true)
  {
    // Up
    cout << "Please set the GPSw (CH" << tobas_navio_ros::kRcChannelGPSw + 1
         << ") to UP and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    gpsw_up = readRCInput(tobas_navio_ros::kRcChannelGPSw);

    // Down
    cout << "Please set the GPSw (CH" << tobas_navio_ros::kRcChannelGPSw + 1
         << ") to DOWN and press Enter:";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    gpsw_down = readRCInput(tobas_navio_ros::kRcChannelGPSw);

    if (abs(gpsw_up - gpsw_down) < kPeriodDiffThreshold)
    {
      TOBAS_ERROR("The signals on GPSw channel are too close. Please retry.");
      continue;
    }

    break;
  }

  // Configに保存
  tobas_std::PropertyTree pt(tobas_navio_ros::kConfigPath);

  pt.put(tobas_navio_ros::kConfigKey_RcRollLeft, roll_left);
  pt.put(tobas_navio_ros::kConfigKey_RcRollRight, roll_right);
  pt.put(tobas_navio_ros::kConfigKey_RcPitchUp, pitch_up);
  pt.put(tobas_navio_ros::kConfigKey_RcPitchDown, pitch_down);
  pt.put(tobas_navio_ros::kConfigKey_RcYawLeft, yaw_left);
  pt.put(tobas_navio_ros::kConfigKey_RcYawRight, yaw_right);
  pt.put(tobas_navio_ros::kConfigKey_RcThrustUp, thrust_up);
  pt.put(tobas_navio_ros::kConfigKey_RcThrustDown, thrust_down);
  pt.put(tobas_navio_ros::kConfigKey_RcModeProgram, mode_program);
  pt.put(tobas_navio_ros::kConfigKey_RcModeStabilize, mode_stabilize);
  pt.put(tobas_navio_ros::kConfigKey_RcModeAcrobat, mode_acrobat);
  pt.put(tobas_navio_ros::kConfigKey_RcEStopOn, estop_up);
  pt.put(tobas_navio_ros::kConfigKey_RcEStopOff, estop_down);
  pt.put(tobas_navio_ros::kConfigKey_RcGPSwOn, gpsw_up);
  pt.put(tobas_navio_ros::kConfigKey_RcGPSwOff, gpsw_down);

  pt.save();
  cout << "Calibration finished. The result is saved to '" << tobas_navio_ros::kConfigPath << "'."
       << endl;
}

double RCInputCalibrator::readRCInput(const size_t& channel)
{
  // RC入力を取得
  int period_sum = 0;
  for (size_t _ = 0; _ < kDataCount; ++_)
  {
    if (rcin_.read(channel) < 0)
      throw runtime_error("Failed to read RC input.");
    period_sum += rcin_.getPeriod();
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
}  // namespace tobas_calibration
