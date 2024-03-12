#include <cstdio>
#include <memory>
#include <vector>
#include <unistd.h>

#include <Common/Util.h>
#include <Navio2/ADC.h>

#define READ_FAILED -1

int main()
{
  if (check_apm())
    return 1;

  ADC adc;
  adc.initialize();
  std::vector<float> results(adc.kChannelCount, 0.);
  while (true)
  {
    for (size_t i = 0; i < adc.kChannelCount; ++i)
    {
      results[i] = adc.read(i);
      if (results[i] == READ_FAILED)
        return EXIT_FAILURE;
      printf("A%zu: %.4fV ", i, results[i] / 1000);
    }
    printf("\n");

    usleep(500000);
  }

  return 0;
}
