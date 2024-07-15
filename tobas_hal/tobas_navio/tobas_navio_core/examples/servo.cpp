#include <iostream>
#include <unistd.h>

#include <tobas_navio_core/pwm.hpp>
#include <tobas_navio_core/util.hpp>

#define SERVO_MIN 1250  // us
#define SERVO_MAX 1750  // us

#define PWM_OUTPUT 0

int main(int, char* argv[])
{
  navio::PWM pwm;

  if (navio::checkAPM())
    return EXIT_FAILURE;

  if (getuid())
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);

  if (!(pwm.initialize(PWM_OUTPUT)))
    return EXIT_FAILURE;

  pwm.setFrequency(PWM_OUTPUT, 50);

  if (!(pwm.enable(PWM_OUTPUT)))
    return EXIT_FAILURE;

  while (true)
  {
    if (!pwm.setDutyCycle(PWM_OUTPUT, SERVO_MIN))
      fprintf(stderr, "Failed to set PWM duty cycle.\n");
    sleep(1);
    if (!pwm.setDutyCycle(PWM_OUTPUT, SERVO_MAX))
      fprintf(stderr, "Failed to set PWM duty cycle.\n");
    sleep(1);
  }

  return EXIT_SUCCESS;
}
