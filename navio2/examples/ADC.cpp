#include <cstdio>
#include <memory>
#include <vector>
#include <unistd.h>

#include <Common/Util.h>
#include <Navio2/ADC_Navio2.h>

#define READ_FAILED -1

int main()
{
  if (check_apm())
    return 1;

  ADC_Navio2 adc;
  adc.initialize();
  std::vector<float> results(adc.get_channel_count(), 0.);
  while (true)
  {
    for (int i = 0; i < adc.get_channel_count(); ++i)
    {
      results[i] = adc.read(i);
      if (results[i] == READ_FAILED)
        return EXIT_FAILURE;
      printf("A%d: %.4fV ", i, results[i] / 1000);
    }
    printf("\n");

    usleep(500000);
  }

  return 0;
}
