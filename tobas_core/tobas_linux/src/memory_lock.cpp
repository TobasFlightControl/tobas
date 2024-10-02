#include <iostream>
#include <cstring>
#include <memory>
#include <vector>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/resource.h>

#include "../include/tobas_linux/memory_lock.hpp"

using namespace std;

namespace linux
{
int lockMemory()
{
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
  {
    cerr << "mlockall failed." << endl;
    return -1;
  }

  // Turn off malloc trimming
  if (mallopt(M_TRIM_THRESHOLD, -1) == 0)
  {
    cerr << "mallopt for trim threshold failed." << endl;
    munlockall();
    return -1;
  }

  // Turn off mmap usage
  if (mallopt(M_MMAP_MAX, 0) == 0)
  {
    cerr << "mallopt for mmap failed." << endl;
    mallopt(M_TRIM_THRESHOLD, 1 << 17);
    munlockall();
    return -1;
  }

  return 0;
}

int lockAndPrefaultDynamic()
{
  if (lockMemory() != 0)
    return -1;

  struct rusage usage;
  size_t page_size = sysconf(_SC_PAGESIZE);
  getrusage(RUSAGE_SELF, &usage);
  vector<char*> prefaulters;
  size_t prev_minflts = usage.ru_minflt;
  size_t prev_majflts = usage.ru_majflt;
  size_t encountered_minflts = 1;
  size_t encountered_majflts = 1;

  // Prefault until you see no more pagefaults
  while (encountered_minflts > 0 || encountered_majflts > 0)
  {
    char* ptr;
    try
    {
      ptr = new char[64 * page_size];
      memset(ptr, 0, 64 * page_size);
    }
    catch (const bad_alloc& e)
    {
      cerr << "Caught exception: " << e.what() << endl;
      cerr << "Unlocking memory and continuing." << endl;
      for (auto& prefaulter : prefaulters)
        delete[] prefaulter;

      mallopt(M_TRIM_THRESHOLD, 1 << 17);
      mallopt(M_MMAP_MAX, 1 << 16);
      munlockall();
      return -1;
    }
    prefaulters.push_back(ptr);
    getrusage(RUSAGE_SELF, &usage);
    size_t current_minflt = usage.ru_minflt;
    size_t current_majflt = usage.ru_majflt;
    encountered_minflts = current_minflt - prev_minflts;
    encountered_majflts = current_majflt - prev_majflts;
    prev_minflts = current_minflt;
    prev_majflts = current_majflt;
  }

  for (auto& prefaulter : prefaulters)
    delete[] prefaulter;

  return 0;
}

int lockAndPrefaultDynamic(size_t process_max_dynamic_memory)
{
  if (lockMemory() != 0)
    return -1;

  void* buf = nullptr;
  const auto pg_sz = sysconf(_SC_PAGESIZE);
  const auto res = posix_memalign(&buf, pg_sz, process_max_dynamic_memory);
  if (res != 0)
  {
    cerr << "proc rt init mem aligning failed: " << strerror(errno) << endl;
    return -1;
  }
  memset(buf, 0, process_max_dynamic_memory);
  free(buf);

  return 0;
}
}  // namespace linux
