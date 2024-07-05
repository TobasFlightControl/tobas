#include <memory>
#include <cstdio>
#include <iostream>
#include <unistd.h>

#include <tobas_navio_core/util.hpp>
#include <tobas_navio_core/rc_input.hpp>

#define READ_FAILED -1

using namespace std;

int main()
{
  if (navio::checkAPM())
    return EXIT_FAILURE;

  navio::RCInput rcin;
  rcin.initialize();

  while (true)
  {
    for (int ch = 0; ch < 14; ++ch)
    {
      int period = rcin.read(ch);
      if (period == READ_FAILED)
        return EXIT_FAILURE;
      cout << "Channel: " << ch << ", Period: " << period << endl;
    }
    cout << endl;
    sleep(1);
  }

  return EXIT_SUCCESS;
}
