#include <iostream>
#include <cstring>
#include <memory>
#include <vector>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/resource.h>

#include "../include/tobas_linux/memory_lock.hpp"
#include "../include/tobas_linux/errer.hpp"

using namespace std;

namespace linux
{
bool lockMemory()
{
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    cerr << "mlockall failed: " << strError() << endl;
    return false;
  }

  // Turn off malloc trimming
  if (mallopt(M_TRIM_THRESHOLD, -1) == 0) {
    cerr << "mallopt for trim threshold failed: " << strError() << endl;
    munlockall();
    return false;
  }

  // Turn off mmap usage
  if (mallopt(M_MMAP_MAX, 0) == 0) {
    cerr << "mallopt for mmap failed: " << strError() << endl;
    mallopt(M_TRIM_THRESHOLD, 1 << 17);
    munlockall();
    return false;
  }

  return true;
}

bool lockAndPrefaultDynamic()
{
  if (!lockMemory()) {
    return false;
  }

  struct rusage usage;
  size_t page_size = sysconf(_SC_PAGESIZE);
  getrusage(RUSAGE_SELF, &usage);
  vector<char*> prefaulters;
  size_t prev_minflts = usage.ru_minflt;
  size_t prev_majflts = usage.ru_majflt;
  size_t encountered_minflts = 1;
  size_t encountered_majflts = 1;

  // Prefault until you see no more pagefaults
  while (encountered_minflts > 0 || encountered_majflts > 0) {
    char* ptr;
    try {
      ptr = new char[64 * page_size];
      memset(ptr, 0, 64 * page_size);
    }
    catch (const bad_alloc& e) {
      cerr << "Caught exception: " << e.what() << endl;
      cerr << "Unlocking memory and continuing." << endl;
      for (auto& prefaulter : prefaulters) {
        delete[] prefaulter;
      }

      mallopt(M_TRIM_THRESHOLD, 1 << 17);
      mallopt(M_MMAP_MAX, 1 << 16);
      munlockall();
      return false;
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

  for (auto& prefaulter : prefaulters) {
    delete[] prefaulter;
  }

  return true;
}

bool lockAndPrefaultDynamic(size_t process_max_dynamic_memory)
{
  if (!lockMemory()) {
    return false;
  }

  void* buf = nullptr;
  const auto pg_sz = sysconf(_SC_PAGESIZE);
  if (posix_memalign(&buf, pg_sz, process_max_dynamic_memory) != 0) {
    cerr << "proc rt init mem aligning failed: " << strError() << endl;
    return false;
  }

  memset(buf, 0, process_max_dynamic_memory);
  free(buf);

  return true;
}
}  // namespace linux
