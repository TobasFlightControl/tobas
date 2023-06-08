#include <unistd.h>
#include <Navio2/RCOutput_Navio2.h>

#include <dh_ros_tools/console_message.hpp>

#include "../../include/tobas_real/esc_calibration.hpp"
#include "../../include/tobas_real/constants.hpp"

#define SLEEP_TIME_HIGH 3  // [s]
#define SLEEP_TIME_LOW 4   // [s]

using namespace std;

namespace tobas_real
{
int calibrateEscs()
{
  // Chech authority
  if (getuid())
  {
    rosError("Not root.");
    return -1;
  }

  // Initialize PWM handler
  RCOutput_Navio2 pwm;
  for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
  {
    if (!(pwm.initialize(channel)))
    {
      rosError("Failed to initialze PWM for channel " << channel << ".");
      return -1;
    }
    if (!pwm.set_frequency(channel, kPwmFrequency))
    {
      rosError("Failed to set frequency for channel " << channel << ".");
      return -1;
    }
    if (!(pwm.enable(channel)))
    {
      rosError("Failed to enable PWM for channel " << channel << ".");
      return -1;
    }
  }

  // Calibrate
  rosInfo("Step 1: Send maximum throttle command for " << SLEEP_TIME_HIGH << "seconds.");
  for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
  {
    if (!pwm.set_duty_cycle(channel, kPwmMax))
    {
      rosError("Failed to set high duty cycle for channel " << channel << ".");
      return -1;
    }
  }
  sleep(SLEEP_TIME_HIGH);

  rosInfo("Step 2: Send minimum throttle command for " << SLEEP_TIME_LOW << "seconds.");
  for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
  {
    if (!pwm.set_duty_cycle(channel, kPwmMin))
    {
      rosError("Failed to set low duty cycle for channel " << channel << ".");
      return -1;
    }
  }
  sleep(SLEEP_TIME_LOW);

  // Finish
  rosInfo("Calibration finished. Now all the ESCs should be calibrated.");
  return 0;
}
}  // namespace tobas_real
