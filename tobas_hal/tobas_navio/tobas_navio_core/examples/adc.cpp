#include <cstdio>
#include <memory>
#include <vector>
#include <unistd.h>

#include <tobas_navio_core/util.hpp>
#include <tobas_navio_core/adc.hpp>

#define READ_FAILED -1

using namespace navio;

int main()
{
  if (checkAPM())
    return 1;

  ADC adc;
  adc.initialize();
  std::vector<float> results(adc.channelCount(), 0.);
  while (true)
  {
    for (size_t i = 0; i < adc.channelCount(); ++i)
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
