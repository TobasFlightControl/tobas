#include <tobas_time_tools/frequency_measure.hpp>

int main()
{
  tobas::tim::FrequencyMeasure freq_measure;

  while (true) {
    freq_measure.count();
  }
}
