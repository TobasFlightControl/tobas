#include "../include/tobas_linux/process_settings.hpp"

#include <cstring>
#include <string>
#include <stdexcept>
#include <iostream>

#include <tobas_std_tools/cmdline_parser.hpp>

#include "../include/tobas_linux/realtime.hpp"
#include "../include/tobas_linux/memory_lock.hpp"

using namespace std;

namespace linux
{
bool ProcessSettings::init(int argc, char* argv[])
{
  if (tobas_std::commandLineOptionExists(argv, argv + argc, "-h")) {
    printUsage();
    return false;
  }

  if (tobas_std::commandLineOptionExists(argv, argv + argc, kOptionLockMemory)) {
    const auto option = tobas_std::getCommandLineOption(argv, argv + argc, kOptionLockMemorySize);
    lock_memory_ = strcmp(option, "true") == 0 ? true : false;
  }

  if (tobas_std::commandLineOptionExists(argv, argv + argc, kOptionLockMemorySize)) {
    lock_memory_size_mb_ = stoi(tobas_std::getCommandLineOption(argv, argv + argc, kOptionLockMemorySize));
    if (lock_memory_size_mb_ > 0) {
      lock_memory_ = true;
    }
  }

  if (tobas_std::commandLineOptionExists(argv, argv + argc, kOptionPriority)) {
    process_priority_ = stoi(tobas_std::getCommandLineOption(argv, argv + argc, kOptionPriority));
  }

  if (tobas_std::commandLineOptionExists(argv, argv + argc, kOptionCPUAffinity)) {
    cpu_affinity_ = stoi(tobas_std::getCommandLineOption(argv, argv + argc, kOptionCPUAffinity));
  }

  return true;
}

bool ProcessSettings::configureProcess()
{
  // Set the priority of this thread to the maximum safe value,
  // and set its scheduling policy to a deterministic (real-time safe) algorithm.
  if (process_priority_ > 0 && process_priority_ < 99) {
    if (!setThisProcessPriority(process_priority_, SCHED_RR)) {
      return false;
    }
  }

  if (cpu_affinity_ > 0) {
    if (!setThisProcessCPUAffinity(cpu_affinity_)) {
      return false;
    }
  }

  if (lock_memory_) {
    if (lock_memory_size_mb_ > 0) {
      if (!lockAndPrefaultDynamic(lock_memory_size_mb_ * (1 << 20))) {
        return false;
      }
    }
    else {
      if (!lockAndPrefaultDynamic()) {
        return false;
      }
    }
  }

  return true;
}

void ProcessSettings::printUsage()
{
  cout << "\t[" << kOptionLockMemory << " lock memory]" << endl
       << "\t[" << kOptionLockMemorySize << " lock a fixed memory size in MB]" << endl
       << "\t[" << kOptionPriority << " set process real-time priority]" << endl
       << "\t[" << kOptionCPUAffinity << " set process cpu affinity]" << endl
       << "\t[-h]" << endl;
}
}  // namespace linux
