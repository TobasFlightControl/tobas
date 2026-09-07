// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/process_settings.hpp"

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include <tobas_std_tools/cmdline_parser.hpp>

#include "tobas_linux/memory_lock.hpp"
#include "tobas_linux/realtime.hpp"

namespace tobas
{
namespace linux
{
namespace
{
constexpr char kOptionLockMemory[] = "--lock-memory";
constexpr char kOptionLockMemorySize[] = "--lock-memory-size";
constexpr char kOptionPriority[] = "--priority";
constexpr char kOptionCPUAffinity[] = "--cpu-affinity";
}  // namespace

bool ProcessSettings::init(int argc, char* argv[])
{
  if (st::commandLineOptionExists(argv, argv + argc, "-h")) {
    printUsage();
    return false;
  }

  if (st::commandLineOptionExists(argv, argv + argc, kOptionLockMemory)) {
    const auto option = st::getCommandLineOption(argv, argv + argc, kOptionLockMemorySize);
    lock_memory_ = std::strcmp(option, "true") == 0 ? true : false;
  }

  if (st::commandLineOptionExists(argv, argv + argc, kOptionLockMemorySize)) {
    lock_memory_size_mb_ = std::stoi(st::getCommandLineOption(argv, argv + argc, kOptionLockMemorySize));
    if (lock_memory_size_mb_ > 0) {
      lock_memory_ = true;
    }
  }

  if (st::commandLineOptionExists(argv, argv + argc, kOptionPriority)) {
    process_priority_ = std::stoi(st::getCommandLineOption(argv, argv + argc, kOptionPriority));
  }

  if (st::commandLineOptionExists(argv, argv + argc, kOptionCPUAffinity)) {
    cpu_affinity_ = std::stoi(st::getCommandLineOption(argv, argv + argc, kOptionCPUAffinity));
  }

  return true;
}

bool ProcessSettings::configureProcess()
{
  // Set the requested real-time priority and use a deterministic scheduling policy.
  if (process_priority_ > 0 && process_priority_ < 99) {
    if (!setThisProcessPriority(process_priority_, SCHED_RR)) {
      return false;
    }
  }

  if (cpu_affinity_ > 0) {
    if (!setThisProcessCpuAffinity(cpu_affinity_)) {
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
  std::cout << "\t[" << kOptionLockMemory << " lock memory]" << std::endl
            << "\t[" << kOptionLockMemorySize << " lock a fixed memory size in MB]" << std::endl
            << "\t[" << kOptionPriority << " set process real-time priority]" << std::endl
            << "\t[" << kOptionCPUAffinity << " set process cpu affinity]" << std::endl
            << "\t[-h]" << std::endl;
}
}  // namespace linux
}  // namespace tobas
