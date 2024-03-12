#include <memory>
#include <cstdio>
#include <iostream>
#include <unistd.h>

#include <navio2/Util.h>
#include <navio2/RCInput.h>

#define READ_FAILED -1

using namespace std;
using namespace navio;

int main()
{
  if (check_apm())
    return 1;

  RCInput rcin;
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

  return 0;
}
