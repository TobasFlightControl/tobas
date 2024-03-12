#include <unistd.h>
#include <unistd.h>
#include <memory>

#include <navio2/util.hpp>
#include <navio2/pwm.hpp>

#define SERVO_MIN 1250 /*uS*/
#define SERVO_MAX 1750 /*uS*/

#define PWM_OUTPUT 0
using namespace navio;

int main(int, char* argv[])
{
  PWM pwm;

  if (check_apm())
  {
    return 1;
  }

  if (getuid())
  {
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
  }

  if (!(pwm.initialize(PWM_OUTPUT)))
  {
    return 1;
  }

  pwm.setFrequency(PWM_OUTPUT, 50);

  if (!(pwm.enable(PWM_OUTPUT)))
  {
    return 1;
  }

  while (true)
  {
    if (!pwm.setDutyCycle(PWM_OUTPUT, SERVO_MIN))
    {
      fprintf(stderr, "Failed to set PWM duty cycle.\n");
    }
    sleep(1);
    if (!pwm.setDutyCycle(PWM_OUTPUT, SERVO_MAX))
    {
      fprintf(stderr, "Failed to set PWM duty cycle.\n");
    }
    sleep(1);
  }

  return 0;
}
