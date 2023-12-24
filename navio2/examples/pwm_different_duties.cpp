#include <memory>
#include <iostream>
#include <unistd.h>

#include <Common/Util.h>
#include <Navio2/PWM.h>
#include <Navio2/RCOutput_Navio2.h>

#define SERVO_RAIL_SIZE 14
#define PWM_FREQUENCY 50    // [Hz]
#define PWM_DISARM 900      // [us]
#define PWM_MIN 1000        // [us]
#define PWM_MAX 2000        // [us]
#define DISARM_DURATION 3.  // [s]
#define INTERVAL 0.1        // [s]

using namespace std;

int main(int, char** argv)
{
  // Check superuser authority
  if (check_apm())
  {
    return 1;
  }
  if (getuid())
  {
    cerr << "Not root. Please launch like this: sudo " << argv[0] << endl;
    return 1;
  }

  // Initialize PWM handler
  RCOutput_Navio2 pwm;
  for (uint32_t channel = 0; channel < SERVO_RAIL_SIZE; ++channel)
  {
    if (!(pwm.initialize(channel)))
    {
      cerr << "Failed to initialze PWM on channel " << channel << endl;
      return 1;
    }
    if (!pwm.setFrequency(channel, PWM_FREQUENCY))
    {
      cerr << "Failed to set frequency on channel " << channel << endl;
      return 1;
    }
    if (!(pwm.enable(channel)))
    {
      cerr << "Failed to enable PWM on channel " << channel << endl;
      return 1;
    }
  }

  // Send disarm command
  cout << "Send disarm command for " << DISARM_DURATION << " seconds." << endl;
  for (int _ = 0; _ < DISARM_DURATION / INTERVAL; ++_)
  {
    for (uint32_t channel = 0; channel < SERVO_RAIL_SIZE; ++channel)
    {
      if (!pwm.setDutyCycle(channel, PWM_DISARM))
      {
        cerr << "Failed to set disarm duty cycle on channel " << channel << endl;
        return 1;
      }
    }
    usleep(INTERVAL * 1e+6);
  }

  // If commands are sent immediately after disarming, the motors will rotate.
  cout << "Start to send PWM commands." << endl;

  // Servo control loop
  while (true)
  {
    for (uint32_t channel = 0; channel < SERVO_RAIL_SIZE; ++channel)
    {
      const double rate = static_cast<double>(channel) / static_cast<double>(SERVO_RAIL_SIZE);
      const double period = PWM_MIN + (PWM_MAX - PWM_MIN) * rate;
      if (!pwm.setDutyCycle(channel, period))
      {
        cerr << "Failed to set PWM duty cycle on channel " << channel << endl;
        return 1;
      }
    }
    usleep(INTERVAL * 1e+6);
  }

  return 0;
}
