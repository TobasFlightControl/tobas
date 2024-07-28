#include <memory>
#include <unistd.h>

#include <tobas_navio_core/util.hpp>
#include <tobas_navio_core/led.hpp>

int main(int, char* argv[])
{
  if (navio::checkAPM())
    return EXIT_FAILURE;

  if (getuid())
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);

  navio::Led led;
  if (!led.initialize())
    return EXIT_FAILURE;

  while (true)
  {
    led.setColor(navio::Colors::Green);
    printf("LED is green\n");
    sleep(1);

    led.setColor(navio::Colors::Cyan);
    printf("LED is cyan\n");
    sleep(1);

    led.setColor(navio::Colors::Blue);
    printf("LED is blue\n");
    sleep(1);

    led.setColor(navio::Colors::Magenta);
    printf("LED is magenta\n");
    sleep(1);

    led.setColor(navio::Colors::Red);
    printf("LED is red\n");
    sleep(1);

    led.setColor(navio::Colors::Yellow);
    printf("LED is yellow\n");
    sleep(1);
  }

  return EXIT_SUCCESS;
}
