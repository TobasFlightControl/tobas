#include <unistd.h>

#include <iostream>

#include <tobas_linux/schedule.hpp>

using namespace std;

int main()
{
  if (!linux::setRealtimePriorityFIFO(50)) {
    return EXIT_FAILURE;
  }

  pause();

  return EXIT_SUCCESS;
}
