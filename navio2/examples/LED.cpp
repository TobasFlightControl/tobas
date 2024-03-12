#include <memory>
#include <unistd.h>

#include <Common/Util.h>
#include <Navio2/Led.h>

int main(int, char* argv[])
{
  if (check_apm())
    return 1;

  if (getuid())
    fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);

  Led led;
  if (!led.initialize())
    return EXIT_FAILURE;

  while (true)
  {
    led.setColor(Colors::Green);
    printf("LED is green\n");
    sleep(1);

    led.setColor(Colors::Cyan);
    printf("LED is cyan\n");
    sleep(1);

    led.setColor(Colors::Blue);
    printf("LED is blue\n");
    sleep(1);

    led.setColor(Colors::Magenta);
    printf("LED is magenta\n");
    sleep(1);

    led.setColor(Colors::Red);
    printf("LED is red\n");
    sleep(1);

    led.setColor(Colors::Yellow);
    printf("LED is yellow\n");
    sleep(1);
  }

  return 0;
}
